#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "ps2_link_guard.h"

#define TIMEOUT_MS 100U
#define RECONNECT_FRAMES 3U

static void connect_neutral(ps2_link_guard_t *guard, uint32_t start_ms)
{
	assert(PS2_LinkGuard_Update(guard, 1U, 1U, start_ms,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(PS2_LinkGuard_Update(guard, 1U, 1U, start_ms + 10U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(PS2_LinkGuard_Update(guard, 1U, 1U, start_ms + 20U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_PROCESS_FRAME);
	assert(guard->online == 1U);
}

static void test_startup_and_timeout(void)
{
	ps2_link_guard_t guard;

	PS2_LinkGuard_Init(&guard, 0U);
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 10U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	connect_neutral(&guard, 20U);

	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 130U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_WAIT);
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 140U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(guard.online == 0U);
	assert(guard.failsafe_count == 1U);
}

static void test_reconnect_requires_neutral(void)
{
	ps2_link_guard_t guard;

	PS2_LinkGuard_Init(&guard, 0U);
	assert(PS2_LinkGuard_Update(&guard, 1U, 0U, 0U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(PS2_LinkGuard_Update(&guard, 1U, 0U, 10U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(guard.valid_streak == 0U);
	connect_neutral(&guard, 20U);
}

static void test_tick_wraparound(void)
{
	ps2_link_guard_t guard;
	const uint32_t near_wrap = UINT32_MAX - 30U;

	PS2_LinkGuard_Init(&guard, near_wrap);
	connect_neutral(&guard, near_wrap);
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 50U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_WAIT);
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 90U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
}

static void test_receiver_frame_loss_triggers_timeout(void)
{
	ps2_link_guard_t guard;

	PS2_LinkGuard_Init(&guard, 0U);
	connect_neutral(&guard, 0U);

	/* A missing or malformed receiver frame must trip the timeout. */
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 110U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_WAIT);
	assert(PS2_LinkGuard_Update(&guard, 0U, 0U, 120U,
		TIMEOUT_MS, RECONNECT_FRAMES) == PS2_LINK_APPLY_FAILSAFE);
	assert(guard.online == 0U);
	assert(guard.failsafe_count == 1U);
}

int main(void)
{
	test_startup_and_timeout();
	test_reconnect_requires_neutral();
	test_tick_wraparound();
	test_receiver_frame_loss_triggers_timeout();
	puts("ps2_link_guard tests passed");
	return 0;
}
