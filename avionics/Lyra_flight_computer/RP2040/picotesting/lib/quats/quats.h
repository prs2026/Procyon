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

        int32_t w,x,y,z; 
        Quaternion(float ww, float xx, float yy, float zz){
            w = ww*10000;
            x = xx*10000;
            y = yy*10000;
            z = zz*10000;
        }

        Quaternion(){

        }

        void normalize(){
            float d = (sqrt(pow(w,2)+pow(x,2)+pow(y,2)+pow(z,2)))/10000;
            w /= d;
            x /= d;
            y /= d;
            z /= d;
        }

        void setquat(int aa,int ii,int jj,int kk){
            w = aa;
            x = ii;
            y = jj;
            z = kk;
        }


        


};


Quaternion addquat(Quaternion q1, Quaternion q2){
    Quaternion result;
    result.w = q1.w + q2.w;
    result.x = q1.x + q2.x;
    result.y = q1.y + q2.y;
    result.z = q1.z + q2.z;
    return result;
}

Quaternion multquat(Quaternion q1, Quaternion q2){
    Quaternion result;
    return result;
}


#endif // quats
