#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "../../User/APP/gamepad_link_protocol.h"

static const int MC02_UART_RX_PIN = 16;
static const int MC02_UART_TX_PIN = 17;
static const uint32_t MC02_UART_BAUD = 115200;
static const uint32_t MC02_FRAME_PERIOD_MS = 10;

HardwareSerial mc02Serial(2);

// GameSir Nova Lite (first generation) uses this stable public BLE address.
XboxSeriesXControllerESP32_asukiaaa::Core xboxController(
    "3a:c3:d6:d6:1b:c1");

String xbox_string()
{
  String str = String(xboxController.xboxNotif.btnY) + "," +
               String(xboxController.xboxNotif.btnX) + "," +
               String(xboxController.xboxNotif.btnB) + "," +
               String(xboxController.xboxNotif.btnA) + "," +
               String(xboxController.xboxNotif.btnLB) + "," +
               String(xboxController.xboxNotif.btnRB) + "," +
               String(xboxController.xboxNotif.btnSelect) + "," +
               String(xboxController.xboxNotif.btnStart) + "," +
               String(xboxController.xboxNotif.btnXbox) + "," +
               String(xboxController.xboxNotif.btnShare) + "," +
               String(xboxController.xboxNotif.btnLS) + "," +
               String(xboxController.xboxNotif.btnRS) + "," +
               String(xboxController.xboxNotif.btnDirUp) + "," +
               String(xboxController.xboxNotif.btnDirRight) + "," +
               String(xboxController.xboxNotif.btnDirDown) + "," +
               String(xboxController.xboxNotif.btnDirLeft) + "," +
               String(xboxController.xboxNotif.joyLHori) + "," +
               String(xboxController.xboxNotif.joyLVert) + "," +
               String(xboxController.xboxNotif.joyRHori) + "," +
               String(xboxController.xboxNotif.joyRVert) + "," +
               String(xboxController.xboxNotif.trigLT) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};

static uint8_t scaleAxis(uint16_t raw, uint8_t neutral)
{
  static const uint16_t deadbandMin = 30720U;
  static const uint16_t deadbandMax = 34815U;

  if (raw >= deadbandMin && raw <= deadbandMax)
  {
    return neutral;
  }
  return (uint8_t)(((uint32_t)raw * 255U) / 65535U);
}

static uint16_t getButtonBits()
{
  uint16_t buttons = 0U;
  if (xboxController.xboxNotif.btnA) buttons |= GAMEPAD_BUTTON_A;
  if (xboxController.xboxNotif.btnB) buttons |= GAMEPAD_BUTTON_B;
  if (xboxController.xboxNotif.btnX) buttons |= GAMEPAD_BUTTON_X;
  if (xboxController.xboxNotif.btnY) buttons |= GAMEPAD_BUTTON_Y;
  if (xboxController.xboxNotif.btnLB) buttons |= GAMEPAD_BUTTON_LB;
  if (xboxController.xboxNotif.btnRB) buttons |= GAMEPAD_BUTTON_RB;
  if (xboxController.xboxNotif.btnSelect) buttons |= GAMEPAD_BUTTON_SELECT;
  if (xboxController.xboxNotif.btnStart) buttons |= GAMEPAD_BUTTON_START;
  if (xboxController.xboxNotif.btnXbox) buttons |= GAMEPAD_BUTTON_XBOX;
  if (xboxController.xboxNotif.btnShare) buttons |= GAMEPAD_BUTTON_SHARE;
  if (xboxController.xboxNotif.btnLS) buttons |= GAMEPAD_BUTTON_LS;
  if (xboxController.xboxNotif.btnRS) buttons |= GAMEPAD_BUTTON_RS;
  if (xboxController.xboxNotif.btnDirUp) buttons |= GAMEPAD_BUTTON_DIR_UP;
  if (xboxController.xboxNotif.btnDirRight) buttons |= GAMEPAD_BUTTON_DIR_RIGHT;
  if (xboxController.xboxNotif.btnDirDown) buttons |= GAMEPAD_BUTTON_DIR_DOWN;
  if (xboxController.xboxNotif.btnDirLeft) buttons |= GAMEPAD_BUTTON_DIR_LEFT;
  return buttons;
}

static void sendMc02Frame(bool controllerReady)
{
  static uint8_t sequence = 0U;
  uint8_t frame[GAMEPAD_LINK_FRAME_SIZE] = {0U};
  uint16_t buttons = controllerReady ? getButtonBits() : 0U;

  frame[GAMEPAD_LINK_INDEX_SOF_0] = GAMEPAD_LINK_SOF_0;
  frame[GAMEPAD_LINK_INDEX_SOF_1] = GAMEPAD_LINK_SOF_1;
  frame[GAMEPAD_LINK_INDEX_VERSION] = GAMEPAD_LINK_VERSION;
  frame[GAMEPAD_LINK_INDEX_SEQUENCE] = sequence++;
  frame[GAMEPAD_LINK_INDEX_BUTTONS_LOW] = (uint8_t)(buttons & 0xFFU);
  frame[GAMEPAD_LINK_INDEX_BUTTONS_HIGH] = (uint8_t)(buttons >> 8U);

  if (controllerReady)
  {
    frame[GAMEPAD_LINK_INDEX_LX] = scaleAxis(xboxController.xboxNotif.joyLHori, 127U);
    frame[GAMEPAD_LINK_INDEX_LY] = scaleAxis(xboxController.xboxNotif.joyLVert, 128U);
    frame[GAMEPAD_LINK_INDEX_RX] = scaleAxis(xboxController.xboxNotif.joyRHori, 127U);
    frame[GAMEPAD_LINK_INDEX_RY] = scaleAxis(xboxController.xboxNotif.joyRVert, 128U);
    frame[GAMEPAD_LINK_INDEX_STATUS] = GAMEPAD_LINK_STATUS_CONTROLLER_CONNECTED;
  }
  else
  {
    frame[GAMEPAD_LINK_INDEX_LX] = 127U;
    frame[GAMEPAD_LINK_INDEX_LY] = 128U;
    frame[GAMEPAD_LINK_INDEX_RX] = 127U;
    frame[GAMEPAD_LINK_INDEX_RY] = 128U;
  }

  frame[GAMEPAD_LINK_INDEX_CRC] =
    GamepadLink_Crc8(frame, GAMEPAD_LINK_INDEX_CRC);
  mc02Serial.write(frame, GAMEPAD_LINK_FRAME_SIZE);
}

/*
支持四种振动模式
left：上左电机动
right：上右电机动
center：下左电机和下右电机一起动，频率高力量小
shake：下左电机和下右电机一起动，频率低力量大

测试结果：
四种模式都可以调振动力度
下左电机和下右电机是绑定的，只能一起动，但是提供了两种振动模式，个人猜测是两种模式的原理是给电机不同的电压
可以随意搭配使用，但center和shake一起用的话执行的应该是shake
*/

// 配置参考
// repo.v.select.center = 0;
// repo.v.select.left = 0;
// repo.v.select.right = 0;
// repo.v.select.shake = 0;
// repo.v.power.center = 0; // x% power
// repo.v.power.left = 0;
// repo.v.power.right = 30;
// repo.v.power.shake = 0;
// repo.v.timeActive = 0; // 振动 x/100 秒，最大2.56秒(uint8_t)
// repo.v.timeSilent = 0;   // 静止 x/100 秒
// repo.v.countRepeat = 0;  // 循环次数 x+1

// 官方例程
void demoVibration()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  Serial.println("full power for 1 sec");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = true;
  repo.v.select.left = false;
  repo.v.select.right = false;
  repo.v.select.shake = false;
  repo.v.power.center = 30; // 30% power
  repo.v.timeActive = 50;   // 0.5 second
  Serial.println("run center 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = false;
  repo.v.select.left = true;
  repo.v.power.left = 30;
  Serial.println("run left 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.left = false;
  repo.v.select.right = true;
  repo.v.power.right = 30;
  Serial.println("run right 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.right = false;
  repo.v.select.shake = true;
  repo.v.power.shake = 30;
  Serial.println("run shake 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.shake = false;
  repo.v.select.center = true;
  repo.v.power.center = 50;
  repo.v.timeActive = 20;
  repo.v.timeSilent = 20;
  repo.v.countRepeat = 2;
  Serial.println("run center 50\% power in 0.2 sec 3 times");
  xboxController.writeHIDReport(repo);
  delay(2000);
}

// 振动反馈，根据扳机按压力度调整振动力度
void demoVibration_2()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  String str_1;
  repo.setAllOff();
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.power.left = (uint8_t)((float)xboxController.xboxNotif.trigLT / (float)TrigMax * 100);
  repo.v.power.right = (uint8_t)((float)xboxController.xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.timeActive = 50;
  xboxController.writeHIDReport(repo);
  str_1 = String(repo.v.power.left) + "," + String(repo.v.power.right) + "\n";

  Serial.print(str_1);
  delay(50);
}

void setup()
{
  Serial.begin(115200);
  mc02Serial.begin(MC02_UART_BAUD, SERIAL_8N1,
                   MC02_UART_RX_PIN, MC02_UART_TX_PIN);
  Serial.println("Starting NimBLE Client");
  xboxController.begin();
}

void loop()
{
  static unsigned long lastStatusAt = 0;
  static unsigned long lastMc02FrameAt = 0;
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
      if (millis() - lastStatusAt >= 1000)
      {
        Serial.println("waiting for first notification");
        lastStatusAt = millis();
      }
    }
    else
    {
      Serial.print(xbox_string());
      // demoVibration();
      // demoVibration_2();
    }
  }
  else
  {
    if (millis() - lastStatusAt >= 1000)
    {
      Serial.println("not connected");
      lastStatusAt = millis();
    }
    if (xboxController.getCountFailedConnection() > 2)
    {
      ESP.restart();
    }
  }
  if (millis() - lastMc02FrameAt >= MC02_FRAME_PERIOD_MS)
  {
    const bool controllerReady = xboxController.isConnected() &&
      !xboxController.isWaitingForFirstNotification();
    sendMc02Frame(controllerReady);
    lastMc02FrameAt = millis();
  }
  delay(10);
}
