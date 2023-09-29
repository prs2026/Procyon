#if !defined(quats)
#define quats

//#include <Arduino.h>
#include <cmath>
/*
The multiplication with 1 of the basis elements i, j, and k is defined by the fact that 1 is a multiplicative identity, that is,

i 1 = 1 i = i , j 1 = 1 j = j , k 1 = 1 k = k .

The products of other basis elements are

i 2 = j 2 = k 2 = − 1 , i j = − j i = k , j k = − k j = i , k i = − i k = j .

Combining these rules,

i j k = − 1.
*/

struct Vector3float
{
    float x;
    float y;
    float z;
};


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

        Quaternion normalize(){
            Quaternion result;
            float d = (sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2)));
            result.w = w/d;
            result.x = x/d;
            result.y = y/d;
            result.z = z/d;
            return result;
        }

        void setquat(int _a,int _i,int _j,int _k){
            w = _a;
            x = _i;
            y = _j;
            z = _k;
        }

        Quaternion conjugate(){
            Quaternion result;
            result.w = w;
            result.x = -x;
            result.y = -y;
            result.z = -z;
            return result;
        }
        


        Quaternion div(float scalar){
            Quaternion result;
            result.w = w/scalar;
            result.x = x/scalar;
            result.y = y/scalar;
            result.z = z/scalar;
            return result;
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





        
};

Quaternion axisangletoquat(float theta,Quaternion axis){
    theta = theta * (3.14159/180);
    theta = theta/2;
    axis.normalize();
    Quaternion q;
    q.w = cos(theta);
    q.x = axis.x*sin(theta);
    q.y = axis.y*sin(theta);
    q.z = axis.z*sin(theta);
    return q;
}

Quaternion inv(Quaternion q){
    Quaternion result;
    result = q.conjugate().div(pow(q.magnitude(),2));
    return result;
}


Quaternion rotate(Quaternion torotate, Quaternion axis,float _theta){ // rotate torotate quaterion around quaterion axis by theta
    //_theta = _theta; // idk why this is needed but it works when i do this?
    Quaternion result;
    Quaternion q = axisangletoquat(_theta,axis); // construct the rotation quaternion from the input axis and theta values

    //Serial.println("quaternion to rotate by");
    //printquat(q);

    Quaternion _q = inv(q);
    result = (q*torotate)*_q;// rotate the original quaternion by the rotatino quaternion
    return result;
}

Quaternion intergrategyros(Quaternion prevstate,Vector3float gyromes,float deltatime){
    Quaternion qdelta(1,0,0,0);
    // assign vector3 to quat so i can use functions
    Quaternion gyromesquat;
    gyromesquat.x = gyromes.x;
    gyromesquat.y = gyromes.y;
    gyromesquat.z = gyromes.z;
    float gyromag = gyromesquat.magnitude();
    gyromesquat = gyromesquat.normalize();
    // 
    qdelta.w = deltatime*gyromag;
    qdelta = qdelta.normalize();
    prevstate = prevstate*qdelta;
    return prevstate;
}



#endif // quats
