#ifndef ARDUINO_ADXL375_ADXL375_H
#define ARDUINO_ADXL375_ADXL375_H

#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "protocol.h"
#include "ADXL375_register.h"
#include "ADXL375_constants.h"
#include "Vector.h"
#include "DynamicFifo.h"

#define ADXL375_ACC_CONVERSION_FACTOR 0.00981 * 49

class ADXL375 {
protected:

    protocol *device;

public:
    /*
     * @brief Constructor overload for I2C protocol
     * @param pipe
     * @param freq
     */
    ADXL375(TwoWire *pipe, uint32_t freq);

    /*
     * @brief Constructor overload for SPI protocol
     * @param chipSelect
     * @param spi
     * @param settings
     */
    ADXL375(byte chipSelect, SPIClass& spi, uint32_t freq);

    /*
     * @brief begin the device
     */
    void begin() {
        device->protocol_begin();
        default_configuration();
        // any other set up etc

    }

    byte who_am_i();

    uint8_t set_spi_mode(bool enable);

    uint8_t set_INTERRUPT(int bit_num, bool set_pin, ADXL375_INTERRUPTS interrupt, bool enable);

    uint8_t set_low_DATA_RATE(ADXL375_Low_power_ODR rate);

    /**
     * Set the device bandwidth and output data rate. Default is 100Hz ADXL375_ODR.
     *
     * The selected output data rate must be appropriate for the communication protocol and frequency selected.
     *
     * Selecting an output data rate that is too high for the communication speed may result in samples being discarded
     * @param rate The new Output Data Rate to set
     * @return 0 if success, else error code
     */
    uint8_t set_data_rate(ADXL375_ODR rate);

    uint8_t set_measure_mode(bool enable);

    uint8_t ac_coupled_mode(bool enable);

    uint8_t set_fifo_mode(ADXL375_FIFO_MODES mode);

    uint8_t default_configuration();

    /**
     * Run the self test procedure as described in the datasheet page 29.
     * @return The self test result (z axis). From testing this seems to range between 50-60.
     */
    float self_test();

    Vector<double, 3> get_accel();

    protocol* get_device() {
        return device;
    }

    /**
     * Write to all registers their default values.
     * @return 0 if success, else error code
     */
    uint8_t reset();
};





//uint8_t set_adxl_ODR(ADXL375_ODR rate);
//uint8_t low_power_mode();
//uint8_t auto_sleep_mode();
//uint8_t standby_mode();
//uint8_t set_FIFO_mode(ADXL375_FIFO_MODES mode);
//uint8_t adxl_self_test();
//
//
//
//class Activity{
//
//    public:
//    //- Value of zero is true any other value is false is the convention
//
//    uint8_t THRESH_SHOCK(bool enable);//Will return true or false
//
//    uint8_t ACT_INACT_CTL(bool enable);//Will return true or false
//
//    uint8_t ACT_SHOCK_STATUS(bool enable);
//    // Will be used to validate if the shock and activity returned by the above functions in the class
//
//
//};
//
// uint8_t BW_RATE(bool enable); // Controls the device bandwidth and output data rate
// uint8_t POWER_CTL(bool enable);//Controls if the sensor is measuring, sleeping or awake based on the activity class output
//
////class Measure_Data{
//
//    //public:
//    uint8_t DATA_FORMAT(bool enable);//Controls the presentation of data to register 0x32 through 0x37
//    uint8_t OUTPUT_DATA();//This class can be called for each axes to get output for each axes
////}
//
//    uint8_t get_offset(bool enable);
////enables offset calibrations to print out the offset measurements of all the axis
////by taking an average of 10 samples at 100 HZ by using Register 0x1E, Register 0x1F, and Register 0x20
//    //returns the offset value based on the register specified.
//
//
//   //uint8_t calculate_acc_data();
//    //Calculates acceleration data for any axes based on the register specified
//
//
//
//

#endif










