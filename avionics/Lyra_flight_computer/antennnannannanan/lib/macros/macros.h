#if !defined(MACROSH)
#define MACROSH

#define CE 14
#define CSN 13

#define SCK 10
#define MOSI 11
#define MISO 12

byte radioaddress[][7] = {"flight","ground"};

struct Vector3float
{
    float x;
    float y;
    float z;
};

struct Vector3int{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Quatstruct{
    float w;
    float x;
    float y;
    float z;
};

struct IMUdata{
    Vector3float accel;
    Vector3float gyro;
    float temp;
};

struct BAROdata{
    float pressure;
    float altitude;
    float temp;
    float verticalvel;
    float maxrecordedalt;
};

struct MAGdata{
    Vector3float gauss;
    Vector3float utesla;
};

union navpacket
{
    struct
    {
        uint32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        IMUdata imudata;
        BAROdata barodata;
        MAGdata magdata;
        //Vector3float pos;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        //Vector3float vel;
        
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];

};

union mpstate{
    struct{
        uint32_t errorflag;
        uint32_t uptime;
        uint32_t state;
        navpacket navsysstate;
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];
    uint8_t data8[sizeof(r)/sizeof(uint8_t)];
};

union telepacket{
    struct 
    {
        uint8_t checksum;
        Vector3int orientationeuler;
        Vector3int accel;
        Vector3int gyro;
        int16_t altitude;
        int16_t verticalvel;
        uint32_t uptime;
        uint8_t errorflagmp;
        uint8_t errorflagnav;
        uint8_t state;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)/sizeof(uint8_t)];

};



#endif // MACROSH
