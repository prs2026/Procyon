#if !defined(quats)
#define quats

#include <Arduino.h>
#include <cmath>
/*
The multiplication with 1 of the basis elements i, j, and k is defined by the fact that 1 is a multiplicative identity, that is,

i 1 = 1 i = i , j 1 = 1 j = j , k 1 = 1 k = k .

The products of other basis elements are

i 2 = j 2 = k 2 = − 1 , i j = − j i = k , j k = − k j = i , k i = − i k = j .

Combining these rules,

i j k = − 1.
*/



class Quaternion // class containing the quaternion values as well as various functions to operate on them
{
    public:

        float w,x,y,z; 
        Quaternion(float _w, float _x, float _y, float _z){
            w = _w;
            x = _x;
            y = _y;
            z = _z;
        }

        Quaternion(){

        }

        float magnitude(){
            float d = (sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2)));
            return d;
        }

        void normalize(){
            float d = (sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2)));
            w /= d;
            x /= d;
            y /= d;
            z /= d;
        }

        void setquat(int _a,int _i,int _j,int _k){
            w = _a;
            x = _i;
            y = _j;
            z = _k;
        }

        

        Quaternion operator+(const Quaternion& q2){
            Quaternion result;
            result.w = w + q2.w;
            result.x = x + q2.x;
            result.y = y + q2.y;
            result.z = z + q2.z;
            return result;
        }

        Quaternion operator*(const Quaternion& q2){
            Quaternion result;
            result.w = (-x * q2.x - y * q2.y - z * q2.z + w * q2.w);
            result.x = ( x * q2.w + y * q2.z - z * q2.y + w * q2.x);
            result.y = (-x * q2.z + y * q2.w + z * q2.x + w * q2.y);
            result.z = ( x * q2.y - y * q2.x + z * q2.w + w * q2.z);
            return result;
        }

        Quaternion operator*(const float s){
            Quaternion result;
            result.w = (w*s);
            result.x = (x*s);
            result.y = (y*s);
            result.z = (z*s);
            return result;
        }

        Quaternion conjugate(){
            Quaternion result;
            result.x = -x;
            result.y = -y;
            result.z = -z;
            return result;
        }

        
};

Quaternion rotate(Quaternion torotate, Quaternion axis,float theta){ // rotate torotate quaterion around quaterion axis by theta
    theta = radians(theta)/2;
    // construct the rotation quaternion from the input axis and theta values
    Quaternion result;
    Quaternion q(1,0,0,0);
    q.w = cos(theta);
    q.x = axis.x*sin(theta);
    q.y = axis.y*sin(theta);
    q.z = axis.z*sin(theta);

    //Serial.println("quaternion to rotate by");
    //printquat(q);
    // rotate the original quaternion by the rotatino quaternion
    Quaternion _q = q.conjugate();
    result = (q*torotate)*_q;
    return result;
}



#endif // quats
