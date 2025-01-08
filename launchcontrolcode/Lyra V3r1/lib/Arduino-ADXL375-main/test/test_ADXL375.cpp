//
// Created by Tom Danvers on 19/12/2022.
//

#include <unity.h>
#include "ADXL375.h"

ADXL375 adxl = ADXL375(3, SPI, 5e6);

#define MRAS_HARDWARE

void setUp() {
    Serial.begin(115200);
    adxl.begin();

#ifdef MRAS_HARDWARE
    // make sure chip select is pulled high for all other devices on MRAS
    pinMode(40, OUTPUT);
    digitalWrite(40, HIGH);

    pinMode(37, OUTPUT);
    digitalWrite(37, HIGH);
#endif
    adxl.reset();
    adxl.default_configuration();
}

void tearDown() {
    // clean stuff up here
}

/* -------------------------------- Tests -------------------------------- */

void test_device_id() {
    uint8_t address = adxl.who_am_i();
    Serial.printf("Device ID: %X (should be E5 for ADXL375)\n", address);
    TEST_ASSERT(address == 0xE5);
}

void test_set_data_rate() {
    // first, test to see that the register contains the expected value at startup
    uint8_t data_rate = adxl.get_device()->read_reg(ADXL375_REGISTER::BW_RATE);

    // mask, expected, actual, line(what is this?), message
    UNITY_TEST_ASSERT_BITS(0b11111111, 0b00001010, data_rate, __LINE__, "BW_RATE should be 0b00001010 at startup");

    // therefore, the current data rate is 100Hz. Lets set it to 200Hz
    adxl.set_data_rate(ADXL375_ODR::ODR_200_HZ);
    data_rate = adxl.get_device()->read_reg(ADXL375_REGISTER::BW_RATE);
    UNITY_TEST_ASSERT_BITS(0b11111111, 0b00001011, data_rate, __LINE__, "BW_RATE should be 0b00001011 at 200Hz");
}

void test_self_test() {
    float result = adxl.self_test();
    Serial.printf("Self test result: %f\n", result);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_device_id);
    RUN_TEST(test_set_data_rate);
    RUN_TEST(test_self_test);

    UNITY_END();
    return 0;
}