#ifndef ARDUINO_ADXL375_ADXL375_CONSTANTS_H
#define ARDUINO_ADXL375_ADXL375_CONSTANTS_H


/* Datasheet: 5.1.2, pg. 37
 bit 0: RW bit. When 0, the data DI(7:0) is written into the device. When 1, the data DO(7:0) from the
 device is read. In latter case, the chip will drive SDO at the start of bit 8.
*/
#define ADXL375_WRITE_BYTE 0b01000000
#define ADXL375_READ_BYTE 0b11000000

#define ADXL375_DEFAULT_I2C_ADDRESS 0x1D // When the ALT ADDRESS pin (Pin 12) is tied high to VDD I/O, the 7-bit I2C address for the device 

enum DEVID{
    ADXL375_Fixed_Device_ID_Code = 0xE5
};


//enum ADXL375_POWER_MODES {
//    LOW_POWER_MODE,
//    AUTOSLEEP_MODE,
//    STANDBY_MODE,
//};


enum ADXL375_BATCHING_DATA_RATE{

  ADXL375_BDR_3200_HZ = 0b1111,
  ADXL375_BDR_1600_HZ = 0b1110,
  ADXL375_BDR_800_HZ = 0b1101,
  ADXL375_BDR_6_25HZ = 0b0110,
  ADXL375_BDR_3_13_HZ = 0b0101,
  ADXL375_BDR_1_56_HZ = 0b0100,
  ADXL375_BDR_0_78_HZ = 0b0011,
  ADXL375_BDR_0_39_HZ = 0b0010,
  ADXL375_BDR_0_20_HZ = 0b0001,
  ADXL375_NO_BATCHING = 0b0000
};

enum ADXL375_Low_power_BDR{
    ADXL375_BDR_400_HZ = 0b1100,
    ADXL375_BDR_100_HZ = 0b1010,
    ADXL375_BDR_50_HZ = 0b1001,
    ADXL375_BDR_25_HZ = 0b1100,
    ADXL375_BDR_12_5_HZ = 0b0111,
};

enum ADXL375_FIFO_MODES { //30 samples collected in each fifo mode except bypass. Bit D5 = 0 links the trigger event of trigger mode to the INT1 pin


    ADXL375_BYPASS_MODE = 0b00000000,
    //In bypass mode, the FIFO buffer is not operational and, therefore, remains empty.

    ADXL375_FIFO_MODE = 0b01111010,

    //In FIFO mode, data from measurements of the x-, y-, and z-axes
    // is stored in the FIFO buffer. When the number of samples in the
    // FIFO buffer equals the level specified by the samples bits of the
    // FIFO_CTL register (Address 0x38), the watermark interrupt is
    // set (see the Watermark Bit section). The FIFO buffer continues
    // to accumulate samples until it is full (32 samples from measurements of the x-, y-, and z-axes) and then stops collecting data.
    // After the FIFO buffer stops collecting data, the device continues
    // to operate; therefore, features such as shock detection can be used
    // after the FIFO buffer is full. The watermark interrupt bit remains
    // set until the number of samples in the FIFO buffer is less than
    // the value stored in the samples bits of the FIFO_CTL register.

    ADXL375_STREAM_MODE = 0b01111001,

    // In stream mode, data from measurements of the x-, y-, and z-axes
    // is stored in the FIFO buffer. When the number of samples in the
    // FIFO buffer equals the level specified by the samples bits of the
    // FIFO_CTL register (Address 0x38), the watermark interrupt is set
    // (see the Watermark Bit section). The FIFO buffer continues to
    // accumulate samples; the buffer stores the latest 32 samples from
    // measurements of the x-, y-, and z-axes, discarding older data as
    // new data arrives. The watermark interrupt bit remains set until
    // the number of samples in the FIFO buffer is less than the value
    // stored in the samples bits of the FIFO_CTL register

    ADXL375_TRIGGER_MODE = 0b01111011,

    // In trigger mode, the FIFO buffer accumulates samples, storing
    // the latest 32 samples from measurements of the x-, y-, and z-axes.
    // After a trigger event occurs, an interrupt is sent to the INT1 or
    // INT2 pin (determined by the trigger bit in the FIFO_CTL register),
    // and the FIFO_TRIG bit (Bit D7) is set in the FIFO_STATUS
    // register (Address 0x39).
    // The FIFO buffer keeps the last n samples (n is the value specified
    // by the samples bits in the FIFO_CTL register) and then operates
    // in FIFO mode, collecting new samples only when the FIFO buffer
    // is not full. A delay of at least 5 µs must elapse between the occurrence of the trigger event and the start of data readback from the
    // FIFO buffer to allow the bufferto discard and retain the necessary
    // samples.
    // Additional trigger events cannot be recognized until the part is
    // reset to trigger mode. To reset the part to trigger mode,
    // 1. If desired, read data from the FIFO buffer (see the Retrieving
    // Data from the FIFO Buffer section).
    // Before resetting the part to trigger mode, read back the
    // FIFO data; placing the device into bypass mode clears the
    // FIFO buffer.
    // 2. Configure the device for bypass mode by setting Bits[D7:D6]
    // at Address 0x38 to 00.
    // 3. Configure the device for trigger mode by setting Bits[D7:D6]
    // at Address 0x38 to 11.



};

//enum INT_MAP not necessary 
// enum INT_MAP {
//   //0 referes to interrupt sending to INT1 pin and 1 reffers to interrupt sending to INT2 pin 
//   D7 = 0, // DATA READY bit
//   D6 =   1, //SINGLE SHOCK bit
//   D5 = 1, //DOUBLE SHOCK bit
//   D4 = 1 // ACTIVITY bit
//   D3 = 1 // INACTIVITY bit
//   D1 = 0 //WATERMARK bit
//   D0 = 0 //OVERRUN bit
//   INT_MAP = 0b01111000 
// };




enum ADXL375_INTERRUPTS {

  ADXL375_Data_Ready = 7, //The DATA_READY bit is set when new data is available and is cleared when no new data is available.
  ADXL375_Single_Shock = 6, // The SINGLE_SHOCK bit is set when a single acceleration event that is greater than the value in the THRESH_SHOCK register  (Address 0x1D) occurs for less time than is specified by the DUR register (Address 0x21)
  ADXL375_Double_Shock = 5,
  //The DOUBLE_SHOCK bit is set when two acceleration events that are greater than the value in the THRESH_SHOCK register (Address 0x1D) occur for less time than is specified by the DUR
  //register (Address 0x21). The second shock event starts after the time specified by the latent register (Address 0x22) but within the time specified by the window register (Address 0x23)
  ADXL375_Activity = 4,
  //The activity bit is set when acceleration greater than the value stored in the THRESH_ACT register (Address 0x24) is experienced on any participating axis. 
  //Participating axes are specified by the ACT_INACT_CTL register (Address 0x27).
  ADXL375_Inactivity = 3,
  //The inactivity bit is set when acceleration less than the value stored in the THRESH_INACT register (Address 0x25) is experienced
  //for more time than is specified by the TIME_INACT register (Address 0x26) on all participating axes. Participating axes are
  //specified by the ACT_INACT_CTL register (Address 0x27). The maximum value for TIME_INACT is 255 sec.
  ADXL375_Watermark = 1,
  //The watermark bit is set when the number of samples in the FIFO buffer equals the value stored in the samples bits (Bits[D4:D0])
  //of the FIFO_CTL register (Address 0x38). The watermark bit is cleared automatically when the FIFO buffer is read and the
  //FIFO contents return to a value below the value specified by the samples bits.
  ADXL375_Overrun = 0,
  // The overrun bit is set when new data replaces unread data. The precise operation of the overrun function depends on the FIFO mode.
  //• In bypass mode, the overrun bit is set when new data replaces unread data in the data registers (Address 0x32 to Address 0x37).
  //• In FIFO mode, stream mode, and trigger mode, the overrun bit is set when the FIFO buffer is full.
  //The overrun bit is automatically cleared when the FIFO buffer contents are read.
};


enum ADXL375_ODR{
  ADXL375_ODR_3200_HZ = 0b1111, ///< 1600Hz Bandwidth   145microA IDD
  ADXL375_ODR_1600_HZ = 0b1110, ///<  800Hz Bandwidth    90microA IDD
  ADXL375_ODR_800_HZ = 0b1101,  ///<  400Hz Bandwidth   140microA IDD
      ///<  200Hz Bandwidth   140microA IDD
  ADXL375_ODR_200_HZ = 0b1011,  ///<  100Hz Bandwidth   140microA IDD
  ADXL375_ODR_100_HZ = 0b1010,  ///<   50Hz Bandwidth   140microA IDD
  ADXL375_ODR_50_HZ = 0b1001,   ///<   25Hz Bandwidth    90microA IDD
  ADXL375_ODR_25_HZ = 0b1000,   ///< 12.5Hz Bandwidth    60microA IDD
  ADXL375_ODR_12_5_HZ = 0b0111, ///< 6.25Hz Bandwidth    50microA IDD
  ADXL375_ODR_6_25HZ = 0b0110,  ///< 3.13Hz Bandwidth    40microA IDD
  ADXL375_ODR_3_13_HZ = 0b0101, ///< 1.56Hz Bandwidth    35microA IDD
  ADXL375_ODR_1_56_HZ = 0b0100, ///< 0.78Hz Bandwidth    35microA IDD
  ADXL375_ODR_0_78_HZ = 0b0011, ///< 0.39Hz Bandwidth    35microA IDD
  ADXL375_ODR_0_39_HZ = 0b0010, ///< 0.20Hz Bandwidth    35microA IDD
  ADXL375_ODR_0_20_HZ = 0b0001, ///< 0.10Hz Bandwidth    35microA IDD
  ADXL375_ODR_0_10_HZ = 0b0000 ///< 0.05Hz Bandwidth    35microA IDD (default value)
};



enum ADXL375_Low_power_ODR{
  ADXL375_Low_ODR_400_HZ = 0b1100,  ///< 200Hz Bandwidth   90microA IDD
  ADXL375_Low_ODR_200_HZ = 0b1011,  ///< 100Hz Bandwidth   60microA IDD
  ADXL375_Low_ODR_100_HZ = 0b1010,  ///< 50Hz Bandwidth    50microA IDD
  ADXL375_Low_ODR_50_HZ = 0b1001,  ///< 25Hz Bandwidth     45microA IDD
  ADXL375_Low_ODR_25_HZ = 0b1100,  ///< 12.5Hz Bandwidth   40microA IDD
  ADXL375_Low_ODR_12_5_HZ = 0b0111, ///< 6.25Hz Bandwidth  35microA IDD
};

 enum ADXL375_Self_test_ODR{
   ADXL375_Self_ODR_800_HZ = 0b1101,  ///<  400Hz Bandwidth   140microA IDD
   ADXL375_Self_ODR_400_HZ = 0b1100,  ///<  200Hz Bandwidth   140microA IDD
   ADXL375_Self_ODR_200_HZ = 0b1011,  ///<  100Hz Bandwidth   140microA IDD
   ADXL375_Self_ODR_100_HZ = 0b1010,  ///<   50Hz Bandwidth   140microA IDD
   ADXL375_Self_ODR_3200_HZ = 0b1111, ///< 1600Hz Bandwidth   145microA IDD
 };


#endif