#if !defined(GENERALLIB)
#define GENERALLIB
// include all macros and data types from macros.h
#include "macros.h"

// functions to convert between my struct data types and eigen data types
// needed becuase eigen types cannot be stored in unions.

inline Vector3float vector3tofloat(Eigen::Vector3d v){
    Vector3float result;
    result.x = v.x();
    result.y = v.y();
    result.z = v.z();
    return result;
}

inline Eigen::Vector3d vectorfloatto3(Vector3float v){
    Eigen::Vector3d result;
    result.x() = v.x;
    result.y() = v.y;
    result.z() = v.z;
    return result;
}

inline Quaterniond quatstructtoeigen(Quatstruct q){
    Quaterniond result;
    result.w() = q.w;
    result.x() = q.x;
    result.y() = q.y;
    result.z() = q.z;
    return result;
}

inline Quatstruct eigentoquatstruct(Quaterniond q){
    Quatstruct result;
    result.w = q.w();
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    return result;
}

// takes in a quatstruct quaterion and spits out a vector3float of euler angles
inline Vector3float quat2euler(Quatstruct inquatstruct){
    Quaterniond quat = quatstructtoeigen(inquatstruct);
    
    Matrix3d R = quat.toRotationMatrix();

    Vector3d euler = R.eulerAngles(0,1,2);
    return vector3tofloat(euler);
    
}

// takes in an mpstate and navpacket and fuses them into a logpacket
inline logpacket preplogentry(mpstate MPstate, navpacket NAVstate){
    logpacket result;
    result.r.checksum1 = 0xAB;
    result.r.checksum2 = 0xCD;
    result.r.MPstate = MPstate;
    result.r.navsysstate = NAVstate;

    return result;
}

// converts a vector to a vector quaternion.
inline Quaterniond vectortoquat(Vector3d vec){
    Quaterniond result;
    result.w() = 0;
    result.x() = vec.x();
    result.y() = vec.y();
    result.z() = vec.z();
    
    return result;
}

//converts a vector quaterion back to a vector
inline Vector3d quattovector(Quaterniond quat){
    Vector3d result;
    result.x() = quat.x();
    result.y() = quat.y();
    result.z() = quat.z();
    
    return result;
}

//takes in an mpstate and a navstate and converts them to a telepacket
inline telepacket statetopacket(mpstate state,navpacket navstate){
    telepacket packet;
    packet.r.checksum = 0x12;
    packet.r.checksum2 = 0x34;
    packet.r.accel.x = int16_t(navstate.r.imudata.accel.x*100);
    packet.r.accel.y = int16_t(navstate.r.imudata.accel.y*100);
    packet.r.accel.z = int16_t(navstate.r.imudata.accel.z*100);

    packet.r.gyro.x = int16_t(navstate.r.imudata.gyro.x*100);
    packet.r.gyro.y = int16_t(navstate.r.imudata.gyro.y*100);
    packet.r.gyro.z = int16_t(navstate.r.imudata.gyro.z*100);

    packet.r.orientationeuler.x = int16_t(navstate.r.orientationeuler.x*100);
    packet.r.orientationeuler.y = int16_t(navstate.r.orientationeuler.y*100);
    packet.r.orientationeuler.z = int16_t(navstate.r.orientationeuler.z*100);

    packet.r.uptime = state.r.uptime;
    packet.r.errorflagmp = state.r.errorflag;
    packet.r.errorflagnav = navstate.r.errorflag;
    packet.r.state = state.r.state;

    packet.r.altitude = int16_t(navstate.r.filtered.alt*10);
    packet.r.verticalvel = int16_t(navstate.r.filtered.vvel*100);
    
    packet.r.status = state.r.status;
    return packet;
}

//prints a number in binary
inline void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}

//scans the i2c bus, returns 1 if none are found, otherwise 0
inline uint8_t scani2c(bool printout){
    byte error, address;
    int nDevices;
     printout ? Serial.print("Scanning..")  : 0;
    
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
        printout ? Serial.print("I2C device found at address 0x") : 0;
        if (address<16)
            Serial.print("0");
        printout ? Serial.print(address,HEX) : 0;
        printout ? Serial.println("  !") : 0;
    
        nDevices++;
        }
        else if (error==4)
        {
        printout ? Serial.print("Unknown error at address 0x") : 0;
        if (address<16)
            printout ? Serial.print("0") : 0;
            printout ? Serial.println(address,HEX) : 0;
        }    
    }
    if (nDevices == 0){
        printout ? Serial.println("No I2C devices found\n") : 0;
        return 1;
    }
    else
        printout ? Serial.println("done\n") : 0;
    
    return 0;
}




#endif // GENERALLIB