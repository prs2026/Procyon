#include "ADXL375.h"
ADXL375::ADXL375(TwoWire *pipe, uint32_t freq) { // constructor for I2C protocol
    device = new I2CProtocol(ADXL375_DEFAULT_I2C_ADDRESS, pipe, freq);
}

ADXL375::ADXL375(byte chipSelect, SPIClass& spi, uint32_t freq){
SPISettings settings = SPISettings(freq, MSBFIRST, SPI_MODE3);
    device = new SPIProtocol(chipSelect, spi, settings, ADXL375_READ_BYTE, ADXL375_WRITE_BYTE);
}

uint8_t ADXL375::set_fifo_mode(ADXL375_FIFO_MODES mode) { //
    byte data = device->read_reg(ADXL375_REGISTER::FIFO_CTL);
    data = data & 0b01111000; // bit 3 must be zero, clear bottom 3 bits
    data = mode | data;
    return device->write_reg(ADXL375_REGISTER::FIFO_CTL, data);
}
uint8_t ADXL375::set_spi_mode(bool enable) { // 1 for 3-wire spi and 0 for 4 wire spi
    byte spi_wires = device->read_reg(ADXL375_REGISTER::DATA_FORMAT);
    setBit(&spi_wires, 7, enable);
    return device->write_reg(ADXL375_REGISTER::DATA_FORMAT, spi_wires);
}

uint8_t ADXL375::set_INTERRUPT(int bit_num, bool set_pin, ADXL375_INTERRUPTS interrupt, bool enable) {

    byte map_data = device->read_reg(ADXL375_REGISTER::INT_MAP) ;
    setBit(&map_data, bit_num, set_pin);//bit num chooses which bit we wanna alter, set pin is 0 or 1 based on interrupt pin you wanna select.
    int a =  device->write_reg(ADXL375_REGISTER::INT_MAP, map_data);
    byte data = device->read_reg(ADXL375_REGISTER::INT_ENABLE);
    setBit(&data, interrupt, enable);
    int b = device->write_reg(ADXL375_REGISTER::INT_ENABLE, data);
    return a | b;
//A setting of 1 for any bit in the INT_ENABLE register enables the specified function to generate interrupts;
// A setting of 0 for any bit in this register prevents the function from generating interrupts
}

uint8_t ADXL375::set_data_rate(ADXL375_ODR rate){
    // set new data rate whilst preserving power mode

    byte bw_rate_reg = device->read_reg(ADXL375_REGISTER::BW_RATE);
    bw_rate_reg &= 0b11110000; // clear bottom 4 bits of existing value
    byte new_rate = rate & 0b00001111; // clean the top 4 bits from new value
    bw_rate_reg = bw_rate_reg | new_rate;
    return device->write_reg(ADXL375_REGISTER::BW_RATE, bw_rate_reg);
}

//USURE HOW TO GO FORWARD
// uint8_t ADXL375::set_autosleep_mode(bool enable){
//     byte Rate_data = device->read_reg(ADXL375_REGISTER::);
//     setBit(&Rate_data,5,1);
//     Rate_data = Rate_data & 0b00000000; // setting bit 5 here to 0 to disable low power mode. there is no need for the above commented line right?
//     Rate_data = rate | Rate_data;
//     return device->write_reg(ADXL375_REGISTER::BW_RATE, Rate_data);
// }

uint8_t ADXL375::set_measure_mode(bool enable){//0 for setting to standby. Sets measure bit in powerctl register to 0 to put it into standby mode
    byte measure_bit = device->read_reg(ADXL375_REGISTER::POWER_CTL);
    setBit(&measure_bit , 3, enable);
    return device->write_reg(ADXL375_REGISTER::POWER_CTL, measure_bit);
}

uint8_t ADXL375::ac_coupled_mode(bool enable){//0 for setting to standby. Sets measure bit in powerctl register to 0 to put it into standby mode
    byte Measure_bit = device->read_reg(ADXL375_REGISTER::POWER_CTL);
    setBit(&Measure_bit , 4, enable);
    return device->write_reg(ADXL375_REGISTER::BW_RATE, Measure_bit);
}

uint8_t ADXL375::default_configuration() {
    reset();

    // configure the device to work to +-400g
    // see https://github.com/adafruit/Adafruit_ADXL375/issues/1 for comments about this (not documented in datasheet..)
    device->write_reg(ADXL375_REGISTER::DATA_FORMAT, 0b00001011);

    // start reading data
    set_measure_mode(true);

    return 0;
}

byte ADXL375::who_am_i() {
    return device->read_reg(ADXL375_REGISTER::DEVID);
}

Vector<double, 3> ADXL375::get_accel(){

    byte buffer[6]= {};
    device->read_regs(ADXL375_REGISTER::DATAX0, buffer, 6);

    Vector<int16_t, 3> raw_accel = {
            (int16_t) ((uint16_t)buffer[1] << 8 | ((uint16_t)buffer[0])),
            (int16_t) ((uint16_t)buffer[3] << 8 | (uint16_t)buffer[2]),
            (int16_t) ((uint16_t)buffer[5] << 8 | (uint16_t)buffer[4])
    };

    Vector<double, 3> accel = ((Vector<double, 3>) raw_accel) * ADXL375_ACC_CONVERSION_FACTOR;

    return accel;
}

uint8_t ADXL375::reset() {
    // GitHub CoPilot I love you.
    device->write_reg(ADXL375_REGISTER::THRESH_SHOCK, 0b00000000);
    device->write_reg(ADXL375_REGISTER::OFSX, 0b00000000);
    device->write_reg(ADXL375_REGISTER::OFSY, 0b00000000);
    device->write_reg(ADXL375_REGISTER::OFSZ, 0b00000000);
    device->write_reg(ADXL375_REGISTER::DUR, 0b00000000);
    device->write_reg(ADXL375_REGISTER::Latent, 0b00000000);
    device->write_reg(ADXL375_REGISTER::Window, 0b00000000);
    device->write_reg(ADXL375_REGISTER::THRESH_ACT, 0b00000000);
    device->write_reg(ADXL375_REGISTER::THRESH_INACT, 0b00000000);
    device->write_reg(ADXL375_REGISTER::TIME_INACT, 0b00000000);
    device->write_reg(ADXL375_REGISTER::ACT_INACT_CTL, 0b00000000);
    device->write_reg(ADXL375_REGISTER::SHOCK_AXES, 0b00000000);
    device->write_reg(ADXL375_REGISTER::BW_RATE, 0b00001010);
    device->write_reg(ADXL375_REGISTER::POWER_CTL, 0b00000000);
    device->write_reg(ADXL375_REGISTER::INT_ENABLE, 0b00000000);
    device->write_reg(ADXL375_REGISTER::INT_MAP, 0b00000000);
    device->write_reg(ADXL375_REGISTER::DATA_FORMAT, 0b00000000);
    device->write_reg(ADXL375_REGISTER::FIFO_CTL, 0b00000000);
    return 0;
}

float ADXL375::self_test() {
//#define PRINT_SELF_TEST
    int n_samples = 100;
    int delay_between_samples = 5;
    // following the procedure on page 29 of the datasheet
    // the device MUST NOT MOVE during the test

    // start by performing a reset of the device
    reset();
    default_configuration();

    // 1. set the data rate to 800Hz
    set_data_rate(ADXL375_ODR::ADXL375_ODR_800_HZ);

    // 2. by default, the device is already in normal power operation (happened in reset())

    //    The self-test response in the x- and y-axes exhibits bimodal
    //    behavior and, therefore, is not always a reliable indicator of
    //    sensor health or potential shift in device sensitivity. For this
    //    reason, perform the self-test check in the z-axis (index 2)

    // 3. collect samples
    float samples_st_off[n_samples];
    for (float &sample: samples_st_off) {
        sample = (float) get_accel()[2];
#ifdef PRINT_SELF_TEST
        Serial.printf("Sample (st off): %f\n", sample);
#endif
        delay(delay_between_samples);
    }

    // 4. store the averaged values
    float average_st_off = {};
    for (float& sample: samples_st_off) {
        average_st_off += sample;
#ifdef PRINT_SELF_TEST
        Serial.printf("Average (st off): %f\n", average_st_off);
#endif
    }
    average_st_off /= 100;
#ifdef PRINT_SELF_TEST
    Serial.printf("Average (st off, done): %f\n", average_st_off);
#endif

    // 5. enable self test
    uint8_t data_format = device->read_reg(ADXL375_REGISTER::DATA_FORMAT);
    setBit(&data_format, 7, 1);
    device->write_reg(ADXL375_REGISTER::DATA_FORMAT, data_format);
    delay(100);

    // 6. collect samples
    float samples_st_on[n_samples];
    for (float &sample: samples_st_on) {
        sample = (float) get_accel()[2];
#ifdef PRINT_SELF_TEST
        Serial.printf("Sample (st on): %f\n", sample);
#endif
        delay(delay_between_samples);
    }

    // 7. store the averaged values
    float average_st_on = {};
    for (float& sample: samples_st_on) {
        average_st_on += sample;
#ifdef PRINT_SELF_TEST
        Serial.printf("Average (st on): %f\n", average_st_on);
#endif
    }
    average_st_on /= 100;
#ifdef PRINT_SELF_TEST
    Serial.printf("Average (st on, done): %f\n", average_st_on);
    Serial.printf("Average (st off, done): %f\n", average_st_off);
#endif

    // 8. disable self test
    data_format = device->read_reg(ADXL375_REGISTER::DATA_FORMAT);
    setBit(&data_format, 7, 0);
    device->write_reg(ADXL375_REGISTER::DATA_FORMAT, data_format);
    delay(100);

    // 9. calculate the difference between the two averages
    float difference = average_st_on - average_st_off;

    return difference;
}


