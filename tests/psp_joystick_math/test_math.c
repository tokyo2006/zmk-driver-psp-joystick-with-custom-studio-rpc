#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "tokyo2006/psp-joystick/psp_joystick_math.h"

static void test_normalize(void) {
    assert(psp_joystick_normalize(2048, 2048, 0, 4095) == 0);
    assert(psp_joystick_normalize(4095, 2048, 0, 4095) == 32767);
    assert(psp_joystick_normalize(0, 2048, 0, 4095) == -32767);
    assert(psp_joystick_normalize(3071, 2048, 0, 4095) == 16375);
    assert(psp_joystick_normalize(5000, 2048, 0, 4095) == 32767);
    assert(psp_joystick_normalize(-100, 2048, 0, 4095) == -32767);
    assert(psp_joystick_normalize(1000, 1000, 0, 4095) == 0);
    assert(psp_joystick_normalize(1000, 1000, 1000, 1000) == 0);
}

static void test_deadzone(void) {
    assert(psp_joystick_apply_deadzone(0, 8) == 0);
    assert(psp_joystick_apply_deadzone(2621, 8) == 0);
    assert(psp_joystick_apply_deadzone(-2621, 8) == 0);
    assert(psp_joystick_apply_deadzone(3000, 8) == 411);
    assert(psp_joystick_apply_deadzone(-3000, 8) == -411);
    assert(psp_joystick_apply_deadzone(32767, 8) == 32767);
    assert(psp_joystick_apply_deadzone(-32767, 8) == -32767);
    assert(psp_joystick_apply_deadzone(5000, 0) == 5000);
}

static void test_curve(void) {
    assert(psp_joystick_apply_curve(0, 100) == 0);
    assert(psp_joystick_apply_curve(0, 300) == 0);
    assert(psp_joystick_apply_curve(32767, 100) == 32767);
    assert(psp_joystick_apply_curve(-32767, 100) == -32767);
    assert(psp_joystick_apply_curve(32767, 150) == 32767);
    assert(psp_joystick_apply_curve(32767, 200) == 32767);
    assert(psp_joystick_apply_curve(32767, 250) == 32767);
    assert(psp_joystick_apply_curve(32767, 300) == 32767);
    assert(psp_joystick_apply_curve(16384, 200) == 8192);
    assert(psp_joystick_apply_curve(-16384, 200) == -8192);
    assert(psp_joystick_apply_curve(10000, 200) <= psp_joystick_apply_curve(20000, 200));
    assert(psp_joystick_apply_curve(10000, 300) <= psp_joystick_apply_curve(20000, 300));
}

static void test_speed_remainder(void) {
    int32_t rem = 0;
    assert(psp_joystick_apply_speed(32767, 8, &rem) == 8);
    assert(rem == 0);

    rem = 0;
    int32_t sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += psp_joystick_apply_speed(1000, 8, &rem);
    }
    assert(sum == 1);

    rem = 0;
    sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += psp_joystick_apply_speed(-1000, 8, &rem);
    }
    assert(sum == -1);

    rem = 0;
    assert(psp_joystick_apply_speed(32767, 0, &rem) == 0);
    assert(rem == 0);
}

static void test_transforms(void) {
    int16_t x = 100;
    int16_t y = 200;

    psp_joystick_apply_transforms(&x, &y, false, true, false);
    assert(x == -100);
    assert(y == 200);

    x = 100;
    y = 200;
    psp_joystick_apply_transforms(&x, &y, false, false, true);
    assert(x == 100);
    assert(y == -200);

    x = 100;
    y = 200;
    psp_joystick_apply_transforms(&x, &y, true, true, true);
    assert(x == -200);
    assert(y == -100);
}

static void test_isqrt(void) {
    assert(psp_joystick_isqrt64(0) == 0);
    assert(psp_joystick_isqrt64(1) == 1);
    assert(psp_joystick_isqrt64(4) == 2);
    assert(psp_joystick_isqrt64(1073676289ULL) == 32767);
}

int main(void) {
    test_normalize();
    test_deadzone();
    test_curve();
    test_speed_remainder();
    test_transforms();
    test_isqrt();
    printf("all math tests passed\n");
    return 0;
}
