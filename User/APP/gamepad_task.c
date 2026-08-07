#include "gamepad_task.h"

#include <string.h>

#include "cmsis_os.h"
#include "gamepad_link_protocol.h"
#include "ps2_link_guard.h"
#include "ps2_task.h"
#include "remote_input_config.h"
#include "usart.h"

#define GAMEPAD_TASK_PERIOD_MS 10U
#define GAMEPAD_UART_RING_SIZE 64U
#define GAMEPAD_UART_RING_MASK (GAMEPAD_UART_RING_SIZE - 1U)

#if __RC_TYPE == RC_GAMESIR_NOVA_LITE

typedef struct
{
  uint8_t bytes[GAMEPAD_LINK_FRAME_SIZE];
  uint8_t index;
} gamepad_stream_parser_t;

extern chassis_t chassis_move;
extern vmc_leg_t right;
extern vmc_leg_t left;

static uint8_t gamepad_uart_ring[GAMEPAD_UART_RING_SIZE];
static volatile uint8_t gamepad_uart_head = 0U;
static volatile uint8_t gamepad_uart_tail = 0U;
static uint8_t gamepad_uart_rx_byte = 0U;
volatile uint32_t gamepad_uart_error_count = 0U;
volatile uint32_t gamepad_uart_overflow_count = 0U;

static void Gamepad_UartStart(void)
{
  gamepad_uart_head = 0U;
  gamepad_uart_tail = 0U;
  gamepad_uart_error_count = 0U;
  gamepad_uart_overflow_count = 0U;
  (void)HAL_UART_Receive_IT(&huart7, &gamepad_uart_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t next_head;

  if (huart->Instance != UART7)
  {
    return;
  }

  next_head = (uint8_t)((gamepad_uart_head + 1U) & GAMEPAD_UART_RING_MASK);
  if (next_head != gamepad_uart_tail)
  {
    gamepad_uart_ring[gamepad_uart_head] = gamepad_uart_rx_byte;
    gamepad_uart_head = next_head;
  }
  else
  {
    gamepad_uart_overflow_count++;
  }

  (void)HAL_UART_Receive_IT(&huart7, &gamepad_uart_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART7)
  {
    return;
  }

  gamepad_uart_error_count++;
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)HAL_UART_Receive_IT(&huart7, &gamepad_uart_rx_byte, 1U);
}

static void Gamepad_ApplyFailsafe(ps2data_t *data, chassis_t *chassis)
{
  uint8_t i;

  chassis->start_flag = 0U;
  chassis->recover_flag = 0U;
  chassis->target_v = 0.0f;
  chassis->v_set = 0.0f;
  chassis->x_set = chassis->x_filter;
  chassis->turn_set = chassis->total_yaw;
  chassis->roll_target = 0.0f;
  chassis->roll_set = 0.0f;
  chassis->leg_set = 0.08f;
  chassis->leg_left_set = chassis->leg_set;
  chassis->leg_right_set = chassis->leg_set;
  chassis->last_leg_set = chassis->leg_set;
  chassis->last_leg_left_set = chassis->leg_left_set;
  chassis->last_leg_right_set = chassis->leg_right_set;
  chassis->count_key = 0U;
  chassis->jump_flag = 0U;
  chassis->jump_time_r = 0U;
  chassis->jump_time_l = 0U;
  chassis->jump_status_r = 0U;
  chassis->jump_status_l = 0U;

  for (i = 0U; i < 2U; i++)
  {
    chassis->wheel_motor[i].wheel_T = 0.0f;
    right.torque_set[i] = 0.0f;
    left.torque_set[i] = 0.0f;
  }

  data->key = 0;
  data->last_key = 0;
  data->lx = 127;
  data->ly = 128;
  data->rx = 127;
  data->ry = 128;
}

static uint8_t Gamepad_StreamFeed(gamepad_stream_parser_t *parser,
                                  uint8_t byte,
                                  uint8_t *frame)
{
  if (parser->index == 0U)
  {
    if (byte == GAMEPAD_LINK_SOF_0)
    {
      parser->bytes[0] = byte;
      parser->index = 1U;
    }
    return 0U;
  }

  if (parser->index == 1U)
  {
    if (byte != GAMEPAD_LINK_SOF_1)
    {
      parser->index = (byte == GAMEPAD_LINK_SOF_0) ? 1U : 0U;
      parser->bytes[0] = GAMEPAD_LINK_SOF_0;
      return 0U;
    }
  }

  parser->bytes[parser->index++] = byte;
  if (parser->index < GAMEPAD_LINK_FRAME_SIZE)
  {
    return 0U;
  }

  parser->index = 0U;
  if (GamepadLink_FrameIsValid(parser->bytes) == 0U)
  {
    return 0U;
  }

  memcpy(frame, parser->bytes, GAMEPAD_LINK_FRAME_SIZE);
  return 1U;
}

static uint8_t Gamepad_ReadLatestFrame(gamepad_stream_parser_t *parser,
                                       uint8_t *frame)
{
  uint8_t received = 0U;
  uint8_t byte;

  while (gamepad_uart_tail != gamepad_uart_head)
  {
    byte = gamepad_uart_ring[gamepad_uart_tail];
    gamepad_uart_tail =
      (uint8_t)((gamepad_uart_tail + 1U) & GAMEPAD_UART_RING_MASK);
    if (Gamepad_StreamFeed(parser, byte, frame) != 0U)
    {
      received = 1U;
    }
  }

  return received;
}

static void Gamepad_FrameToLegacyControl(const uint8_t *frame,
                                         ps2data_t *data)
{
  uint16_t buttons;

  buttons = (uint16_t)frame[GAMEPAD_LINK_INDEX_BUTTONS_LOW] |
            ((uint16_t)frame[GAMEPAD_LINK_INDEX_BUTTONS_HIGH] << 8U);

  data->lx = frame[GAMEPAD_LINK_INDEX_LX];
  data->ly = frame[GAMEPAD_LINK_INDEX_LY];
  data->rx = frame[GAMEPAD_LINK_INDEX_RX];
  data->ry = frame[GAMEPAD_LINK_INDEX_RY];

  /* Reuse the PS2 actions: LB->L2(off), RB->R2(on), A->R1(jump). */
  Handkey = 0xFFFFU;
  if ((buttons & GAMEPAD_BUTTON_LB) != 0U)
  {
    Handkey &= (uint16_t)(~PS2_MOTOR_FORCE_OFF_MASK);
  }
  if ((buttons & GAMEPAD_BUTTON_RB) != 0U)
  {
    Handkey &= (uint16_t)(~PS2_MOTOR_FORCE_ON_MASK);
  }
  data->key = ((buttons & GAMEPAD_BUTTON_A) != 0U) ? PSB_R1 : 0;
}

void Gamepad_Task(void)
{
  gamepad_stream_parser_t parser = {{0U}, 0U};
  ps2_link_guard_t link_guard;
  ps2_link_action_t link_action;
  ps2data_t gamepad_data;
  uint8_t frame[GAMEPAD_LINK_FRAME_SIZE] = {0U};
  uint8_t received_frame;
  uint8_t raw_frame_valid;
  uint32_t now_ms;

  Gamepad_ApplyFailsafe(&gamepad_data, &chassis_move);
  Handkey = 0xFFFFU;
  MX_UART7_Init();
  Gamepad_UartStart();
  now_ms = HAL_GetTick();
  PS2_LinkGuard_Init(&link_guard, now_ms);

  while (1)
  {
    received_frame = Gamepad_ReadLatestFrame(&parser, frame);
    raw_frame_valid = (received_frame != 0U) &&
      ((frame[GAMEPAD_LINK_INDEX_STATUS] &
        GAMEPAD_LINK_STATUS_CONTROLLER_CONNECTED) != 0U);

    if (raw_frame_valid != 0U)
    {
      Gamepad_FrameToLegacyControl(frame, &gamepad_data);
    }

    now_ms = HAL_GetTick();
    link_action = PS2_LinkGuard_Update(&link_guard, raw_frame_valid,
      PS2_ControlsAreNeutral(&gamepad_data), now_ms,
      PS2_FAILSAFE_TIMEOUT_MS, PS2_RECONNECT_VALID_FRAMES);

    ps2_raw_frame_valid = raw_frame_valid;
    ps2_link_online = link_guard.online;
    ps2_last_valid_ms = link_guard.last_valid_ms;
    ps2_failsafe_count = link_guard.failsafe_count;

    if (link_action == PS2_LINK_PROCESS_FRAME)
    {
      PS2_data_process(&gamepad_data, &chassis_move,
        (float)GAMEPAD_TASK_PERIOD_MS / 1000.0f);
    }
    else if (link_action == PS2_LINK_APPLY_FAILSAFE)
    {
      Gamepad_ApplyFailsafe(&gamepad_data, &chassis_move);
      Handkey = 0xFFFFU;
    }

    ps2_motor_force_enabled = chassis_move.start_flag;
    osDelay(GAMEPAD_TASK_PERIOD_MS);
  }
}

#else

void Gamepad_Task(void)
{
  while (1)
  {
    osDelay(1000U);
  }
}

#endif
