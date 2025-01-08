#ifndef ADXL375_REGISTERS_H
#define ADXL375_REGISTERS_H

enum ADXL375_REGISTER{
    DEVID                   = 0x00, // R
    //RESERVED              = 0x01-0x1C
    THRESH_SHOCK            = 0x1D, // R/W
    OFSX                    = 0x1E, // R/W
    OFSY                    = 0x1F, // R/W
    OFSZ                    = 0x20, // R/W
    DUR                     = 0x21, // R/W
    Latent                  = 0x22, // R/W
    Window                  = 0x23, // R/W
    THRESH_ACT              = 0x24, // R/W
    THRESH_INACT            = 0x25, // R/W
    TIME_INACT              = 0x26, // R/W
    ACT_INACT_CTL           = 0x27, // R/W
    SHOCK_AXES              = 0x2A, // R/W
    ACT_SHOCK_STATUS        = 0x2B, // R
    BW_RATE                 = 0x2C, // R/W
    POWER_CTL               = 0x2D, // R/W
    INT_ENABLE              = 0x2E, // R/W
    INT_MAP                 = 0x2F, // R/W
    INT_SOURCE              = 0x30, // R
    DATA_FORMAT             = 0x31, // R/W
    DATAX0                  = 0x32, // R
    DATAX1                  = 0x33, // R
    DATAY0                  = 0x34, // R
    DATAY1                  = 0x35, // R
    DATAZ0                  = 0x36, // R
    DATAZ1                  = 0x37, // R
    FIFO_CTL                = 0x38, // R/W
    FIFO_STATUS             = 0x39, // R
};

#endif //ADXL375_REGISTERS_H