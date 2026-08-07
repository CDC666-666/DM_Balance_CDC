#ifndef __PS2_LINK_GUARD_H
#define __PS2_LINK_GUARD_H

#include <stdint.h>

typedef enum
{
	PS2_LINK_WAIT = 0,
	PS2_LINK_PROCESS_FRAME,
	PS2_LINK_APPLY_FAILSAFE
} ps2_link_action_t;

typedef struct
{
	uint32_t last_valid_ms;
	uint32_t failsafe_count;
	uint8_t online;
	uint8_t valid_streak;
} ps2_link_guard_t;

static __inline void PS2_LinkGuard_Init(ps2_link_guard_t *guard, uint32_t now_ms)
{
	guard->last_valid_ms = now_ms;
	guard->failsafe_count = 0U;
	guard->online = 0U;
	guard->valid_streak = 0U;
}

/*
 * Unsigned subtraction intentionally handles the HAL tick wraparound.
 * frame_valid reports whether the receiver returned a valid protocol frame.
 * A disconnected controller must then provide several safe frames before its
 * commands are accepted again.
 */
static __inline ps2_link_action_t PS2_LinkGuard_Update(ps2_link_guard_t *guard,
	uint8_t frame_valid, uint8_t controls_neutral, uint32_t now_ms,
	uint32_t timeout_ms, uint8_t reconnect_valid_frames)
{
	if (frame_valid != 0U)
	{
		guard->last_valid_ms = now_ms;

		if (guard->online == 0U)
		{
			if (controls_neutral != 0U)
			{
				if (guard->valid_streak < reconnect_valid_frames)
				{
					guard->valid_streak++;
				}

				if ((reconnect_valid_frames == 0U) ||
					(guard->valid_streak >= reconnect_valid_frames))
				{
					guard->online = 1U;
				}
			}
			else
			{
				guard->valid_streak = 0U;
			}
		}

		return (guard->online != 0U) ? PS2_LINK_PROCESS_FRAME : PS2_LINK_APPLY_FAILSAFE;
	}

	guard->valid_streak = 0U;
	if (guard->online == 0U)
	{
		return PS2_LINK_APPLY_FAILSAFE;
	}

	if ((uint32_t)(now_ms - guard->last_valid_ms) >= timeout_ms)
	{
		guard->online = 0U;
		guard->failsafe_count++;
		return PS2_LINK_APPLY_FAILSAFE;
	}

	return PS2_LINK_WAIT;
}

#endif
