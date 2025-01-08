#if !defined(MPCOREHEADER)
#define MPCOREHEADER

#include <navcore.h>
#include <pyrobatt.h>
#include <RPi_Pico_TimerInterrupt.h>

//fs::File logtofile;

NAVCORE NAV;
SERIALPORT port;
RADIO telemetryradio;

PYROCHANNEL P1(1);
PYROCHANNEL P2(2);
PYROCHANNEL P3(3);
PYROCHANNEL P4(4);

RPI_PICO_Timer PyroTimer0(0);

CircularBuffer<logpacket,LOGBUFSIZE> logbuf = {};

bool checkfirepyros(struct repeating_timer *t)
{ 
  (void) t;

    P1.checkfire();
    P2.checkfire();
    P3.checkfire();
    P4.checkfire();

    //Serial.println("checking pyros");

  return true;
}





class MPCORE{
    
    int buf[FLASH_PAGE_SIZE/sizeof(int)];  // One page buffer of ints
    int addr;
    int32_t first_empty_page = -1;


    public:
        
        int radiook = 1;
        mpstate _sysstate;
        int detectiontime = 0;
        uint32_t landedtime = 0;
        bool datamoved = false;
        uint32_t landingdetectiontime = 0;
        uint32_t liftofftime = 0;
        uint32_t missionelasped = 0;
        uint32_t burnouttime = 0;
        bool ready = false;
        bool beepon = 1;
        bool dogpspassthrough = false;




        MPCORE();

        //int32_t _sysstate.errorflag = 0;
        /*
            0 = no error
            1 = serial init failure
            10 = radio init fail
        */
        bool sendserialon = false;
        bool sendtoteleplot = true;
        
        int freqs[6] = {3000,5000,5000,5000,5000,8000};


        struct timings{
            uint32_t logdata;
            uint32_t led;
            uint32_t serial;
            uint32_t sendtelemetry;
            uint32_t beep;
            uint32_t loop;
        };
        timings intervals[6] = {
            {50,1000,50,1000,10000}, // ground idle
            {50,500,100, 1000,800}, // powered ascent
            {50,500,100,1000,800}, // unpowered ascent
            {50,500,100,1000,800}, // ballistic descent
            {50,800,100,1000,800}, //ready to land
            {1000,1500,100,1000,500} // landed
        };
        timings prevtime;
        bool ledstate = false;



        void setuppins();
        int initperipherials();

        void beep();
        void beep(int freq);
        void beep(int freq, unsigned int duration);
        void beepcont();

        void setled(int state);
        
        int flashtest();
        int logdata(mpstate state,navpacket navstate);
        logpacket readdata(uint64_t page,bool serial);
        int logtobuf();

        int movebuftofile();

        int erasedata();
        int flashinit();
        int dumpdata();
        int previewdata();

        int changestate();
        int parsecommand(char input);
        int sendtelemetry();
        int checkforpyros();

        float readbattvoltage();
        int calibrateimus();
        

};

MPCORE::MPCORE(){
    _sysstate.r.state = 0;
    _sysstate.r.errorflag = 0;
    _sysstate.r.pyrosfired = 0b000;
};

void MPCORE::setuppins(){
    pinMode(LEDPIN,OUTPUT);
    pinMode(BUZZERPIN,OUTPUT);
    pinMode(BATT_SENSE,INPUT);
    pinMode(GPSRST,OUTPUT);

    digitalWrite(GPSRST,HIGH);
    digitalWrite(LEDPIN, HIGH);
    
    return;
}

void MPCORE::setled(int state){
    digitalWrite(LEDPIN,state);
    return;
}

void MPCORE::beep(){
    if (beepon){
        tone(BUZZERPIN,4000,200);
        delay(200);
        noTone(BUZZERPIN);
    }
    

    return;
}

void MPCORE::beep(int freq){
    if (beepon){
        tone(BUZZERPIN,freq,200);
        delay(200);
        noTone(BUZZERPIN);
    }
    return;
}

void MPCORE::beep(int freq, unsigned int duration){
    if (beepon)
    {
        tone(BUZZERPIN,freq,duration);
        delay(duration);
        noTone(BUZZERPIN);
    }
return;
}

void MPCORE::beepcont(){
    if (_sysstate.r.pyroscont & 0b1)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b10)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b100)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b1000)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }

}

float MPCORE::readbattvoltage(){
    long raw = analogRead(BATT_SENSE);
    long adjusted  = map(raw,0,1230,0,1023);
    double calculated = double(adjusted)/100;
    return calculated;
}

int MPCORE::flashtest(){
    // Serial.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
    // Serial.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE,DEC));
    // Serial.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
    // Serial.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
    // Serial.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));
    
    // Read the flash using memory-mapped addresses
    // For that we must skip over the XIP_BASE worth of RAM
    // int addr = FLASH_TARGET_OFFSET + XIP_BASE;
    int32_t page = 0;
    int *p;
    while (page < FLASH_FILESYSTEM_SIZE/FLASH_PAGE_SIZE)
    {
        addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        p = (int *)addr;
        // Serial.print("First four bytes of page " + String(page, DEC) );
        // Serial.print("( at 0x" + (String(int(p), HEX)) + ") = ");
        // Serial.println(*p);
        if( *p == -1 && first_empty_page < 0){
            first_empty_page = page;
            Serial.printf(" empty page at %d", +first_empty_page);
            return 0;
        }
        page++;
    }

    
    // *buf = 123456;

    // if (first_empty_page < 0){
    //     Serial.println("Full sector, erasing...");
    //     uint32_t ints = save_and_disable_interrupts();
    //     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    //     first_empty_page = 0;
    //     restore_interrupts (ints);
    // }
    // Serial.println("Writing to page #" + String(first_empty_page, DEC));
    // rp2040.idleOtherCore();
    // uint32_t ints = save_and_disable_interrupts();
    // flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)buf, FLASH_PAGE_SIZE);
    // restore_interrupts (ints);
    // rp2040.resumeOtherCore();
    // Serial.println("out of test");
    return 0;
}



int MPCORE::initperipherials(){
    int error = port.init();
    error ? _sysstate.r.errorflag || 0b1 : _sysstate.r.errorflag;
    error = telemetryradio.init();
    error ? _sysstate.r.errorflag || 0b10 : _sysstate.r.errorflag;
    _sysstate.r.batterystate = readbattvoltage();
    

    if (PyroTimer0.attachInterruptInterval(100 * 1000, checkfirepyros))
    {
      //Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
    }
    else
        Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

    flashtest();

    P1.timeout = 500;
    P2.timeout = 500;
    P3.timeout = 500;
    P4.timeout = 500;

    return 0;
}


int MPCORE::logtobuf(){

    NAV.event ? _sysstate.r.status = _sysstate.r.status | 0b1 : _sysstate.r.status = _sysstate.r.status;
    uint32_t openingtime = micros();
    _sysstate.r.batterystate = readbattvoltage();

    logpacket datatolog = preplogentry(_sysstate,NAV._sysstate);
    logbuf.unshift(datatolog);

    return 0;
}

int MPCORE::movebuftofile(){
    Serial.print("BUF!");
    //fs::File logfile = LittleFS.open("/log.csv", "a+");
    for (int i = 0; i < LOGBUFSIZE; i++)
    {
        if (logbuf.isEmpty())
        {
            Serial.print("BUF EMPTY!");
            return 0;
        }
        
        logpacket datatolog = logbuf.pop();
        datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
        logdata(datatolog.r.MPstate,datatolog.r.navsysstate);
    }
    
    //logfile.close();

    
    return 0;
}

int MPCORE::logdata(mpstate state, navpacket navstate){

    NAV.event ? state.r.status = state.r.status | 0b1 : state.r.status = state.r.status;
    uint32_t openingtime = micros();
    state.r.batterystate = readbattvoltage();
 
    logpacket datatolog = preplogentry(state,navstate);

    datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
    uint8_t logbuf[FLASH_PAGE_SIZE] = {};
    int j = 0;
    // Serial.println("datapacket is: ");
    // for (int i = 0; i < sizeof(logbuf); i++)
    // {
    //     Serial.printf("%x ",datatolog.data[j]);
    //     j++;
    // }
    // Serial.println("");
    // j = 0;
    for (int i = 0; i < sizeof(datatolog.data); i++)
    {
        logbuf[j] = datatolog.data[j];
        j++;
    }
    
    // Serial.println("packet is: ");
    // j = 0;
    // for (int i = 0; i < sizeof(logbuf); i++)
    // {
    //     Serial.printf("%x ",logbuf[j]);
    //     j++;
    // }
    // Serial.println("");

    //Serial.println("Writing to page #" + String(first_empty_page, DEC));
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)logbuf, FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    rp2040.resumeOtherCore();
    
    first_empty_page++;
    
    return 0;
}

logpacket MPCORE::readdata(uint64_t pageadr,bool serial){
    uint8_t *p;
    addr = XIP_BASE + FLASH_TARGET_OFFSET + (pageadr * FLASH_PAGE_SIZE);
    p = (uint8_t *)addr;
    // Serial.printf("Firstbytes of page %d",pageadr);
    // Serial.printf("( at 0x %d ) = ",int(*p));
    // Serial.println(*p);
    int j = 0;
    logpacket readpacket;
    for (int i = 0; i < sizeof(readpacket.data); i++)
    {
        readpacket.data[j] = *(uint8_t *)(addr+j);
        j++;
    }
    // Serial.println("read packet is: ");
    // j = 0;
    // for (int i = 0; i < sizeof(readpacket.data); i++)
    // {
    //     Serial.printf("%x ",readpacket.data[j]);
    //     j++;
    // }
    
    serial ? port.senddata(readpacket.r.MPstate,readpacket.r.navsysstate) : 1 == 1;
    return readpacket;
}

int MPCORE::erasedata(){
    Serial.println("erasing...");
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_FILESYSTEM_SIZE-FLASH_SECTOR_SIZE-5);
    first_empty_page = 0;
    restore_interrupts (ints);
    rp2040.resumeOtherCore();
    return 0;
}

int MPCORE::previewdata(){

    uint32_t entrynum = 0;
    uint32_t page = 0;
    while (page < first_empty_page)
    {
        readdata(page,true);
        page++;
    }

    Serial.println("done");
    return 0;
}

int MPCORE::dumpdata(){
    Serial.println("dumping data to serial");
    Serial.println("newfile");
    Serial.println("index, checksum,uptime mp,uptime nav,missiontime,  errorflag mp,errorflag NAV,  accel x, accel y, accel z, accelworld x, accelworld y, accelworld z, mag x, mag y, mag z, accelhighg x, accelhighg y, accelhighg z, gyro x, gyro y, gyro z, euler x, euler y, euler z, quat w, quat x, quat y, quat z, altitude, presusre, verticalvel,filtered vvel, maxalt, altitudeagl, filtered alt, imutemp, barotemp,state,battstate,pyros fired,pyros cont,pyros state, checksum2");
    uint32_t entrynum = 0;
    logpacket preventry;
    while (entrynum < first_empty_page)
    {
        logpacket readentry = readdata(entrynum,false);
        if (readentry.r.MPstate.r.missiontime < preventry.r.MPstate.r.missiontime)
        {
            Serial.println("newfile");
            Serial.println("index, checksum,uptime mp,uptime nav,missiontime,  errorflag mp,errorflag NAV,  accel x, accel y, accel z, accelworld x, accelworld y, accelworld z, accelhighg x, accelhighg y, accelhighg z, gyro x, gyro y, gyro z, euler x, euler y, euler z, quat w, quat x, quat y, quat z, altitude, presusre, verticalvel,filtered vvel, maxalt, altitudeagl, filtered alt, imutemp, barotemp,state,battstate,pyros fired,pyros cont,pyros state, checksum2");
        }
        
        Serial.printf(
        "%d, 101,"// index checksum,
        "%d,%d,%d,"//uptimes, mission time
        "%d,%d,"//errorflag
        "%f,%f,%f," // accel
        "%f,%f,%f," // accel
        "%f,%f,%f," // accel world
        "%f,%f,%f," // high g accel
        "%f,%f,%f," // gyro
        "%f,%f,%f," // orientation euler"
        "%f,%f,%f,%f," // orientation quat"
        "%f,%f," //altitude, presusre
        "%f,%f," //verticalvel,filtered vvel,
        "%f,%f,%f," // max alt, altitudeagl, filtered alt
        "%f,%f," // temps, imu baro
        "%d,%f,%d,%d,%d, 202\n", //state, battstate, pyros, pyrocont, pyrostate
        entrynum,
        readentry.r.MPstate.r.uptime, 
        readentry.r.navsysstate.r.uptime,

        readentry.r.MPstate.r.missiontime,

        readentry.r.MPstate.r.errorflag, 
        readentry.r.navsysstate.r.errorflag,

        readentry.r.navsysstate.r.imudata.accel.x, 
        readentry.r.navsysstate.r.imudata.accel.y, 
        readentry.r.navsysstate.r.imudata.accel.z,

        readentry.r.navsysstate.r.accelworld.x, 
        readentry.r.navsysstate.r.accelworld.y, 
        readentry.r.navsysstate.r.accelworld.z,

        readentry.r.navsysstate.r.magdata.utesla.x, 
        readentry.r.navsysstate.r.magdata.utesla.y, 
        readentry.r.navsysstate.r.magdata.utesla.z,

        readentry.r.navsysstate.r.adxldata.accel.x, 
        readentry.r.navsysstate.r.adxldata.accel.y, 
        readentry.r.navsysstate.r.adxldata.accel.z,

        readentry.r.navsysstate.r.imudata.gyro.x*(180/M_PI),
        readentry.r.navsysstate.r.imudata.gyro.y*(180/M_PI),
        readentry.r.navsysstate.r.imudata.gyro.z*(180/M_PI),

        readentry.r.navsysstate.r.orientationeuler.x*(180/M_PI), 
        readentry.r.navsysstate.r.orientationeuler.y*(180/M_PI), 
        readentry.r.navsysstate.r.orientationeuler.z*(180/M_PI),

        readentry.r.navsysstate.r.orientationquat.w, 
        readentry.r.navsysstate.r.orientationquat.x,
        readentry.r.navsysstate.r.orientationquat.y, 
        readentry.r.navsysstate.r.orientationquat.z,

        readentry.r.navsysstate.r.barodata.altitude, 
        readentry.r.navsysstate.r.barodata.pressure, 

        readentry.r.navsysstate.r.barodata.verticalvel, 
        readentry.r.navsysstate.r.filtered.vvel, 

        readentry.r.navsysstate.r.barodata.maxrecordedalt, 
        readentry.r.navsysstate.r.barodata.altitudeagl,
        readentry.r.navsysstate.r.filtered.alt,

        readentry.r.navsysstate.r.imudata.temp,
        readentry.r.navsysstate.r.barodata.temp,

        readentry.r.MPstate.r.state,
        readentry.r.MPstate.r.batterystate,
        readentry.r.MPstate.r.pyrosfired,
        readentry.r.MPstate.r.pyroscont,
        readentry.r.MPstate.r.pyrostate
    );
    entrynum++;
    preventry = readentry;
    }
        
    Serial.println("done");
    return 0;
}

int MPCORE::changestate(){

    Vector3d accelvec = vectorfloatto3(NAV._sysstate.r.imudata.accel);
    Vector3d gyrovec  = vectorfloatto3(NAV._sysstate.r.imudata.gyro);
    float accelmag = accelvec.norm();
    if (_sysstate.r.state == 0) // detect liftoff
    {
        
        NAV._sysstate.r.filtered.alt > 8 && _sysstate.r.uptime > 500 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 400)
        {
            _sysstate.r.state = 1;
            detectiontime = millis();
            Serial.println("liftoff");
            liftofftime = millis();
            movebuftofile();
        }
        
    }

    else if (_sysstate.r.state == 1) // detect burnout
    {

        NAV._sysstate.r.accelworld.z < 8 ? detectiontime = detectiontime : detectiontime = millis();
        
        if (millis() - detectiontime >= 200)
        {
            _sysstate.r.state = 2;
            detectiontime = millis();
            Serial.println("burnout");
            burnouttime = millis();
        }
    }

    else if (_sysstate.r.state == 2) // detect appogee
    {
        NAV._sysstate.r.filtered.alt < NAV._sysstate.r.filtered.maxalt *0.99 && NAV._sysstate.r.filtered.vvel < 50 ?  detectiontime = detectiontime : detectiontime = millis();

        if (millis() - detectiontime >= 300)
        {
            _sysstate.r.state = 3;
            detectiontime = millis();
            Serial.println("appogee");
        }
    }

    else if (_sysstate.r.state == 3) // detect chute opening
    {
        

        accelmag > 7 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 500)
        {
            _sysstate.r.state = 4;
            detectiontime = millis();
            Serial.println("chutes out");
        }
    }

    else if (_sysstate.r.state == 4) // detect landing
    {   

        if (abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
        {
                detectiontime = detectiontime;
        }
        else{
            detectiontime = millis();
        }

        if (millis() - detectiontime >= 3000)
        {
            _sysstate.r.state = 5;
            detectiontime = millis();
            landedtime = millis();
            Serial.println("landed");
        }
    }

    if (_sysstate.r.state > 1 && _sysstate.r.state != 5 && abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
    {
        landingdetectiontime = landingdetectiontime;
    }
    else{
        landingdetectiontime = millis();
    }
    if (millis() - landingdetectiontime >= 30000)
        {
            _sysstate.r.state = 5;
            detectiontime = millis();
            landedtime = millis();
            Serial.println("randoland");
    }

    return 0;
}



// 0 = ground idle  1 = powered ascent 2 = unpowered ascent 3 = ballisitic decsent 4 = under chute 5 = landed
int MPCORE::checkforpyros(){

    if (NAV._sysstate.r.filtered.alt < NAV._sysstate.r.filtered.maxalt && _sysstate.r.state >= 3)
    {
        _sysstate.r.pyrosfired =  _sysstate.r.pyrosfired | 0b1;
        //Serial.println("deploying apogee");
        P1.fire();

    }  

    if (NAV._sysstate.r.filtered.alt < 200 && _sysstate.r.state >= 3)
    {
        
        _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b10;
        //Serial.println("deploying main");
        P2.fire();
    } 

    int stagingstate = 0;
    if (_sysstate.r.state == 2){
        stagingstate = 1;
    if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) > 90-20){
        stagingstate = 2;
    if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) < 90+20){
        stagingstate = 3;
    if(abs(NAV._sysstate.r.orientationeuler.y*(180/M_PI)) > 180-20){
        stagingstate = 4;
    if(NAV._sysstate.r.filtered.vvel > 40){
        stagingstate = 5;
    if(NAV._sysstate.r.filtered.alt > 100){
        stagingstate = 6;
    if(millis() - burnouttime > 4000)
    {
        stagingstate = 7;
        _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b100;
        P3.fire();
    }
    }}}}}}
    else
    {
        //_sysstate.r.pyrosfired = _sysstate.r.pyrosfired & 0b11;
    }
    //Serial.printf(">stagingstate: %d\n",stagingstate);
    

    P1.checkfire();
    P2.checkfire();
    P3.checkfire();
    P4.checkfire();

    uint8_t pyrocont = 0;
    pyrocont = pyrocont | P1.getcont();
    pyrocont = pyrocont | P2.getcont()*2;
    pyrocont = pyrocont | P3.getcont()*4;
    pyrocont = pyrocont | P4.getcont()*8;
    _sysstate.r.pyroscont = pyrocont;

    uint8_t pyrostate = 0;
    pyrostate = pyrostate | P1.state();
    pyrostate = pyrostate | P2.state()*2;
    pyrostate = pyrostate | P3.state()*4;
    pyrostate = pyrostate | P4.state()*8;
    _sysstate.r.pyrostate = pyrostate;
    return 0;
}


int MPCORE::parsecommand(char input){
    if (int(input) == 0)
    {
        return 1;
    }
    Serial.println(int(input));
    int32_t page;
    char channel;
    logpacket currentpacket;
    int j = 0;

    if (input == 'l' && _sysstate.r.state < 1){
        Serial.println("put into launch mode");
            _sysstate.r.state = 1;
            detectiontime = millis();
            Serial.println("liftoff");
            liftofftime = millis();
            movebuftofile();
        return 0;
    }

    else if (input == 'a' && (_sysstate.r.state < 4 || _sysstate.r.state >= 6 )){
        _sysstate.r.state = 0;
        return 0;
    }

    switch (input)
    {
    case 's':
        Serial.println("printing data to teleplot");
        sendserialon = !sendserialon;
        NAV.sendtoserial = !NAV.sendtoserial;
        port.sendtoplot = true;
        break;

    case 'w':
        Serial.println("printing data to processing");
        sendserialon = !sendserialon;
        port.sendtoplot = false;
        break;

    case 'b':
        Serial.println("toggle beeping");
        beepon = !beepon;
        break;
    
    case 'X':
        Serial.println("getting new offsets for adxl");
        adxl.getnewoffsets();
        break;

    case 'B':
        Serial.println("dumping databuf");
        j = 0;
        for (int i = 0; i < LOGBUFSIZE; i++)
        {
            currentpacket = logbuf[j];
            port.senddata(currentpacket.r.MPstate,currentpacket.r.navsysstate);
            j++;
        }
        break;

    case 'D':

        dumpdata();
        break;
    
    case 't':
        Serial.println(" hitltesting");
        NAV.hitltime = millis();
        NAV.hitlteston = 1;
        break;
    case 'r':
        page = Serial.parseInt();
        Serial.println("testing");
        readdata(page,true);
        break;
    
    case 'd':
        previewdata();
        break;
    
    case 'o':
        NAV.getpadoffset();
        break;
    
    case 'P':
        if (Serial.read() != 'A')
        {
            return 0;
        }
        channel = Serial.read();
        Serial.print("firing pyro ");
        Serial.println(channel);
        switch (channel)
        {
        case '1':

            P1.fire();
            break;
        
        case '2':
            P2.fire();
            break;
        
        case '3':
            P3.fire();
            break;
        
        case '4':
            P4.fire();
            break;
        
        default:
            break;
        }
        break;
    
    case 'e':
        Serial.println("erasing flash?");
        if (Serial.read() == 'e')
        {
            Serial.println("erasing flash");
            uint32_t erasestarttime = millis();
            uint32_t messagetime = millis();
            while (millis()-erasestarttime < 10000)
            {
                if (millis() - messagetime > 500)
                {
                    Serial.println("ERASING FLASH ------ ERASING FLASH ------ ERASING FLASH ------ TO ABORT ERASION DISCONNECT POWER OR TYPE A CHARACTER");
                    messagetime = millis();
                }
                if (Serial.available())
                {
                    return 1;
                }
                
            }
            
            erasedata();
        }
        break;

    case 'i':
    NAV.getcalibrationdata();
    break;

    case 'y':
    NAV.dumpoffsets();
    break;

    case 'c':
    calibrateimus();
    break;

    case 'g':
    dogpspassthrough = !dogpspassthrough;
    break;
    
    default:
        break;
    }
    
    
    return 0;
}

int MPCORE::sendtelemetry(){
    telepacket packettosend;
    if (radiook = 1)
    {
        packettosend = statetopacket(_sysstate,NAV._sysstate);
        telemetryradio.sendpacket(packettosend);
    }

    return 0;
}

int MPCORE::calibrateimus(){
    imu.bcal = 0.5*(imu.calibrationpos+imu.calibrationneg);
    adxl.bcal = 0.5*(adxl.calibrationpos+imu.calibrationneg);
    return 0;
}



#endif // MPCOREHEADER


