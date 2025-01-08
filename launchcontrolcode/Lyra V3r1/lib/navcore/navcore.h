#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Lyrav2sensors.h>

//#define VERBOSETIMES


// Dimensions of the matrices
#define Nstate 3 // length of the state vector
#define Nobs 2   // length of the measurement vector

// measurement std 
#define n1 0.1 // noise on baro when slow
#define n12 5 // noise on baro when fast (>200m/s as of 4/25)
#define n2 0.0186 // noise on accel
#define n22 5 // noise on accel when descending

// model std 
#define m1 0.01 //alt
#define m2 0.0005 //vel
#define m3 0.1 //accel

KALMAN<Nstate,Nobs> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector

IMU imu;
BARO baro;
ADXL adxl;
MAG magclass;
GPS gps;

class NAVCORE{

    Quatstruct quatadj = {1,0,0,0};
    Quaterniond vectoradj = {0.707,0,0.707,0};

    Quaterniond upsidedownadj = {0,1,0,0};


    const float ALTVAR = 0.01;
    const float VVELVAR = 0.12;
    const float VACCELVAR = 0.6;
    const float ORIENTVAR = 0.4;

    const float ALTNOISE = 0.1;
    const float VVELNOISE = 1.5;
    const float VACCELNOISE = 0.2;
    const float ORIENTNOISE = 0.4;


    
    navpacket prevsysstate;
    uint32_t kfupdatetime;
    uint32_t kfpredicttime;

    uint32_t invertedtime;

    

    

    double accumz = 0;

    uint64_t hitlindex = 0;
    
    

    public:
        uint64_t hitltime = 0;
        uint8_t hitlteston = 0;
        navpacket _sysstate;

        uint8_t state;

        int useaccel = 1;

        int ready = 0;

        int upsidedown = 0;

        int event = 0;

        int sendtoserial = 0;

        void KFinit();

        NAVCORE();
        /*
        0 = no errors 
        1 = i2c devices fail
        10 = accel init fail
        100 = gyro init fail
        1000 = baro init fail
        10000 = adxl init fail
        100000 = mag init fail
        1000000 = gps init fail
        */
        struct timings{
            uint32_t sendpacket;
            uint32_t intergrateorientation;
            uint32_t kfupdate;
            uint32_t looptime;
            uint32_t getdata;
            uint32_t predictkf;
            uint32_t updatekf;
        };
        timings intervals[6] = {
            {50}, // ground idle
            {50}, // powered ascent
            {50}, // unpowered ascent
            {50}, // ballistic descent
            {50}, //ready to land
            {50} // landed
        }; 
        timings prevtime;


        int initi2c();

        uint32_t sensorinit();
        void getsensordata();

        void KFrun();                                                         
        

        Quatstruct intergrategyros(double timestep);

        Quatstruct adjustwithaccel(float alpha);

        void upsidedowncheck();
        
        Vector3float getworldaccel(navpacket _state);

        void getpadoffset();
        void getcalibrationdata();
        void calcnewoffsets();
        void dumpoffsets();
};

void NAVCORE::KFinit(){
    _sysstate.r.filtered.alt = 0;
    _sysstate.r.filtered.vvel = 0;
    _sysstate.r.filtered.vertaccel = 0;
    _sysstate.r.gpsdata.sats = 100;
    
    prevsysstate = _sysstate;
    prevtime.kfupdate = micros();
    _sysstate.r.orientationquat = adjustwithaccel(0);
}

NAVCORE::NAVCORE(){
    _sysstate.r.orientationquat = {1,0,0,0};
    _sysstate.r.errorflag = 0;
    _sysstate.r.filtered.maxalt = 0;
    Vector3d axis;
    axis << 1,0,0;
    Quaterniond tempquat(AngleAxisd(0.5*PI,axis));
    vectoradj = tempquat;

 //evolution matrix
    K.F = { 1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,1.0};
    // measurement matrix
    K.H = { 1.0, 0.0,
            0.0, 0.0,
            0.0, 1.0};
    //measurement covariance matrix.
    K.R = {n1*n1,   0.0,
            0.0, n2*n2};
    // model covariance matrix. 
    K.Q = {m1*m1, 0.0,  0.0,
           0.0, m2*m2, 0.0,
           0.0, 0.0, m3*m3};

    state = 0;
}
/*
0 = no errors 
1 = i2c devices fail
10 = accel init fail
100 = gyro init fail
1000 = baro init fail
10000 = adxl init fail
100000 = mag init fail
*/
int NAVCORE::initi2c(){
    Wire.setSCL(SCL);
    Wire.setSDA(SDA);
    Wire.setClock(10000);
    Wire.begin();
    scani2c(true) ? _sysstate.r.errorflag || 0b1 : _sysstate.r.errorflag;
    return 0;
}

void NAVCORE::upsidedowncheck(){
    if (accumz > 0)
    {
        upsidedown = 0;
        Serial.println("rightsideup");
    }
    else
    {
        upsidedown = 1;
        Serial.println("upsidedown");
    }
    accumz = 0;
    
    return;
}

uint32_t NAVCORE::sensorinit(){
    int imustatus;
    int barostatus;
    int adxlstatus;
    imustatus = imu.init();;
    imustatus == 1 ? _sysstate.r.errorflag || 0b10 : _sysstate.r.errorflag;
    imustatus == 2 ? _sysstate.r.errorflag || 0b100 : _sysstate.r.errorflag;
    barostatus = baro.init();
    barostatus ? _sysstate.r.errorflag || 0b1000 : _sysstate.r.errorflag;
    adxlstatus = adxl.init();
    barostatus ? _sysstate.r.errorflag || 0b10000 : _sysstate.r.errorflag;
    magclass.init();
    //gps.init();
    
    return 0;
}

void NAVCORE::getsensordata(){
    
    uint32_t hitlindex = 0;

    if (hitlteston)
    {
        //Serial.println("hitltesting");
        while ((hitldata[hitlindex][0]*1000)-(timetostart*1000) < millis() - hitltime && hitlteston)
        {
            //Serial.printf("%f,%d\n",hitldata[hitlindex][0]*1000,millis() - hitltime);
            hitlindex++;
            if (hitlindex > (sizeof(hitldata)/sizeof(hitldata[0]))-1)
            {
                hitlteston = 0;
                Serial.println("out of hitltesting");
            }
        }
    }
    

    #if !defined(VERBOSETIMES)
        imu.read(5,hitlteston,hitlindex);
        baro.readsensor(hitlteston,hitlindex);
        adxl.read(hitlteston,hitlindex);
        magclass.readsensor();
        //gps.read();
    #endif // VERBOSETIMES
    
    if (useaccel == 1)
    {
        accumz += imu.data.accel.z;
        if (millis() - invertedtime > 5000 )
        {
            upsidedowncheck();
            invertedtime = millis();
        }
    }

    
    if (upsidedown == 1)
    {
        imu.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.accel)) * upsidedownadj.inverse()) ));
        imu.data.gyro = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.gyro)) * upsidedownadj.inverse()) ));
        adxl.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(adxl.data.accel)) * upsidedownadj.inverse()) ));
        magclass.data.utesla = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(magclass.data.utesla)) * upsidedownadj.inverse()) ));
        magclass.data.gauss = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(magclass.data.gauss)) * upsidedownadj.inverse()) ));
    }
    
    if (imu.data.absaccel > 20 || adxl.data.absaccel > 40)
    {
        event = 1;
    }
    
    

    _sysstate.r.adxldata = adxl.data;
    _sysstate.r.imudata = imu.data;
    _sysstate.r.barodata = baro.data;
    _sysstate.r.magdata = magclass.data;
    _sysstate.r.gpsdata = gps.data;
    
    return;
}

void NAVCORE::KFrun(){
    navpacket extrapolatedsysstate = _sysstate;

    double timestep = (micros() - kfpredicttime)/1e6;

    Quaterniond adjquat = quatstructtoeigen(quatadj).normalized();

    if (_sysstate.r.filtered.vvel >= 200)
    {
        K.R = {n12*n12,   0.0,
                0.0, n2*n2};
    }
    else if (state >= 3)
    {
        K.R = {n1*n1,   0.0,
                0.0, n22*n22};
    }
    else
    {
        K.R = {n1*n1,   0.0,
                0.0, n2*n2};
    }

    obs(0) = _sysstate.r.barodata.altitudeagl;
    obs(1) = _sysstate.r.accelworld.z;

    K.F = {1.0,timestep,0.5*pow(timestep,2),
           0.0, 1.0,timestep,
           0.0, 0.0, 1.0};

    K.update(obs);

    BLA::Matrix<3> newstate = K.getxcopy();

    extrapolatedsysstate.r.covariences.x = K.P(0,0);
    extrapolatedsysstate.r.covariences.y = K.P(1,1);
    extrapolatedsysstate.r.covariences.z = K.P(2,2);

    extrapolatedsysstate.r.filtered.alt = newstate(0); 
    extrapolatedsysstate.r.filtered.vvel = newstate(1); 
    extrapolatedsysstate.r.filtered.vertaccel = newstate(2);
    

    extrapolatedsysstate.r.orientationquat = intergrategyros(timestep);
    extrapolatedsysstate.r.orientationquatadj = eigentoquatstruct(adjquat* (quatstructtoeigen(extrapolatedsysstate.r.orientationquat).normalized() * adjquat.inverse()));
    
    extrapolatedsysstate.r.accelworld = getworldaccel(extrapolatedsysstate);
    extrapolatedsysstate.r.orientationeuler = quat2euler(extrapolatedsysstate.r.orientationquatadj);



    //Serial.printf(">extrap var: %f\n",extrapolatedsysstate.r.confidence.alt);

    kfpredicttime = micros();
    _sysstate = extrapolatedsysstate;
    _sysstate.r.filtered.alt > _sysstate.r.filtered.maxalt ? _sysstate.r.filtered.maxalt = _sysstate.r.filtered.alt : _sysstate.r.filtered.alt;
    // _data.altitudeagl > data.maxrecordedalt ? data.maxrecordedalt = _data.altitudeagl : data.maxrecordedalt = data.maxrecordedalt;

}                                                         

Quatstruct NAVCORE::intergrategyros(double timestep){
    Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
    Vector3d gyro = vectorfloatto3(_sysstate.r.imudata.gyro);
    
    Quaterniond qdelta(AngleAxisd(timestep*gyro.norm(), gyro.normalized()));

    orientationquat = orientationquat * qdelta;

    return eigentoquatstruct(orientationquat.normalized());
}


Quatstruct NAVCORE::adjustwithaccel(float alpha){
    Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
    Vector3d accel = vectorfloatto3(_sysstate.r.imudata.accel);

    Quaterniond accelquat;

    accelquat.x() = accel.x();
    accelquat.y() = accel.y();
    accelquat.z() = accel.z();

    accelquat = orientationquat * accelquat * orientationquat.inverse();

    accel.x() = accelquat.x();
    accel.y() = accelquat.y();
    accel.z() = accelquat.z();
    

    Vector3d accelnorm(accel.normalized());

    float phi = acos(accelnorm.y());

    Vector3d naxis(accelnorm.cross(Vector3d(0,1,0)));

    naxis = naxis.normalized();


    Quaterniond accelrotquat(AngleAxisd((1-alpha)*phi,naxis));

    orientationquat = accelrotquat * orientationquat;
    //Serial.println(">adjustingwithaccel: 1");

    return eigentoquatstruct(orientationquat.normalized());
}


Vector3float NAVCORE::getworldaccel(navpacket _state){
    Vector3d accelvec = vectorfloatto3(_state.r.imudata.accel);
    Quaterniond orientationquat3 = quatstructtoeigen(_state.r.orientationquatadj);//.inverse();

    Quaterniond accelquat2;

    accelquat2.x() = accelvec.x();
    accelquat2.y() = accelvec.y();
    accelquat2.z() = accelvec.z();

    accelquat2 = orientationquat3 * (accelquat2 * orientationquat3.inverse());

    accelquat2 = vectoradj * (accelquat2 * vectoradj.inverse());

    accelvec.x() = accelquat2.x();
    accelvec.y() = accelquat2.y();
    accelvec.z() = accelquat2.z();   
    

    
    Vector3d grav;
    grav << 0,0,9.801;
    Vector3d _accelworld = accelvec-grav;

    return vector3tofloat(_accelworld);

};

void NAVCORE::getpadoffset(){
    baro.getpadoffset();
    _sysstate.r.filtered.maxalt = 0;
}

void NAVCORE::getcalibrationdata(){
    Serial.println("calibrating accels");
    IMUdata imucalidata = imu.readraw(50,1000);
    ADXLdata adxlcalidata = adxl.readraw(50,1000);
    double lowestval,highestval = 0;
    int index = 0;

    if (imucalidata.accel.x < lowestval)
    {
        lowestval = imucalidata.accel.x;
        index = -1;
    }
    if (imucalidata.accel.y < lowestval)
    {
        lowestval = imucalidata.accel.y;
        index = -2;
    }
    if (imucalidata.accel.z < lowestval)
    {
        lowestval = imucalidata.accel.z;
        index = -3;
    }

    if (imucalidata.accel.x > highestval)
    {
        lowestval = imucalidata.accel.x;
        index = 1;
    }
        if (imucalidata.accel.y > highestval)
    {
        lowestval = imucalidata.accel.y;
        index = 2;
    }
        if (imucalidata.accel.z > highestval)
    {
        lowestval = imucalidata.accel.z;
        index = 3;
    }
    
    switch (index)
    {
    case -1:
        imu.calibrationneg.x() = imucalidata.accel.x;
        adxl.calibrationneg.x() = adxlcalidata.accel.x;
        break;
    
    case -2:
        imu.calibrationneg.y() = imucalidata.accel.y;
        adxl.calibrationneg.y() = adxlcalidata.accel.y;
        break;
    
    case -3:
        imu.calibrationneg.z() = imucalidata.accel.z;
        adxl.calibrationneg.z() = adxlcalidata.accel.z;
        break;
    
    case 1:
        imu.calibrationpos.x() = imucalidata.accel.x;
        adxl.calibrationpos.x() = adxlcalidata.accel.x;
        break;
    
    case 2:
        imu.calibrationpos.y() = imucalidata.accel.y;
        adxl.calibrationpos.y() = adxlcalidata.accel.y;
        break;
    
    case 3:
        imu.calibrationpos.z() = imucalidata.accel.z;
        adxl.calibrationpos.z() = adxlcalidata.accel.z;
        break;

    
    
    default:
        break;
    }
    
    return;
}

void NAVCORE::calcnewoffsets(){
    Vector3d imubcal,adxlbcal;
    Matrix3d imuacal;

    imubcal << (imu.calibrationneg + imu.calibrationpos)/2;
    adxlbcal << (imu.calibrationneg + imu.calibrationpos)/2;

    imu.bcal = imubcal;
    adxl.bcal = adxlbcal;
    return;
}

void NAVCORE::dumpoffsets(){
    Serial.printf("BMI: %f,%f,%f \n ADXL: %f,%f,%f",imu.bcal.x(),imu.bcal.y(),imu.bcal.z(),adxl.bcal.x(),adxl.bcal.y(),adxl.bcal.z());
    return;
}

#endif // NAVCOREHEADER