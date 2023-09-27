#if !defined(quats)
#define quats

#include <cmath>
#include <stdio.h>

/*
The multiplication with 1 of the basis elements i, j, and k is defined by the fact that 1 is a multiplicative identity, that is,

i 1 = 1 i = i , j 1 = 1 j = j , k 1 = 1 k = k .

The products of other basis elements are

i 2 = j 2 = k 2 = − 1 , i j = − j i = k , j k = − k j = i , k i = − i k = j .

Combining these rules,

i j k = − 1.
*/


class QuaternionFloat
{
    public:
        float w,x,y,z; 

        QuaternionFloat(float ww, float xx, float yy, float zz){
            w = ww;
            x = xx;
            y = yy;
            z = zz;
        }

        QuaternionFloat(){
            
        }



};



class Quaternion
{
    public:

        __int32 w,x,y,z; 
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

        QuaternionFloat tofloat(){
            QuaternionFloat result;
            result.w = float(w)/10000;
            result.x = float(x)/10000;
            result.y = float(y)/10000;
            result.z = float(z)/10000;
            return result;
        }


};

Quaternion Quaterniontoint(QuaternionFloat q){
    Quaternion result;
    result.w = int(q.w)*10000;
    result.x = int(q.x)*10000;
    result.y = int(q.y)*10000;
    result.z = int(q.z)*10000;
    return result;
}

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

void printquat(Quaternion q1){
    QuaternionFloat q2 = q1.tofloat();
    printf("w = %f,xi = %fi, yj = %fj zk = %fk \n",q2.w,q2.x,q2.y,q2.z);
    
}

void printquat(QuaternionFloat q2){
    printf("w = %f,xi = %fi, yj = %fj zk = %fk \n",q2.w,q2.x,q2.y,q2.z);
}

#endif // quats
