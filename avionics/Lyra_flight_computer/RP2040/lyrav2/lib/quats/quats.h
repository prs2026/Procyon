#if !defined(quats)
#define quats

//#include <Arduino.h>
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

struct Vector3float
{
    float x;
    float y;
    float z;
};


struct Quatstruct{
    double w;
    double x;
    double y;
    double z;
};



class Quaternion // class containing the quaternion values as well as various functions to operate on them
{
    public:

        Quatstruct r; 
        Quaternion(float _w, float _x, float _y, float _z){
            r.w = _w;
            r.x = _x;
            r.y = _y;
            r.z = _z;
        }

        Quaternion(){
            r.w = 0;
            r.x = 0;
            r.y = 0;
            r.z = 0;
        }

        float magnitude(){
            float d = (sqrt((r.w*r.w)+(r.x*r.x)+(r.y*r.y)+(r.z*r.z)));
            return d;
        }

        Quaternion normalize(){
            Quaternion result;
            float d = (sqrt((r.w*r.w)+(r.x*r.x)+(r.y*r.y)+(r.z*r.z)));
            result.r.w = r.w/d;
            result.r.x = r.x/d;
            result.r.y = r.y/d;
            result.r.z = r.z/d;
            return result;
        }

        void setquat(int _a,int _i,int _j,int _k){
            r.w = _a;
            r.x = _i;
            r.y = _j;
            r.z = _k;
        }

        Quaternion conjugate(){
            Quaternion result;
            result.r.w = r.w;
            result.r.x = -r.x;
            result.r.y = -r.y;
            result.r.z = -r.z;
            return result;
        }
        


        Quaternion div(float scalar){
            Quaternion result;
            result.r.w = r.w/scalar;
            result.r.x = r.x/scalar;
            result.r.y = r.y/scalar;
            result.r.z = r.z/scalar;
            return result;
        }

        Quaternion operator+(const Quaternion& q2){
            Quaternion result;
            result.r.w = r.w + q2.r.w;
            result.r.x = r.x + q2.r.x;
            result.r.y = r.y + q2.r.y;
            result.r.z = r.z + q2.r.z;
            return result;
        }

        

        Quaternion operator*(const Quaternion& q2){
            Quaternion result;
            result.r.w = ((r.w * q2.r.w) - (r.x * q2.r.x) - (r.y * q2.r.y) - (r.z * q2.r.z));
            result.r.x = ((r.w * q2.r.x) + (r.x * q2.r.w) + (r.y * q2.r.z) - (r.z * q2.r.y));
            result.r.y = ((r.w * q2.r.y) - (r.x * q2.r.z) + (r.y * q2.r.w) + (r.z * q2.r.x));
            result.r.z = ((r.w * q2.r.z) + (r.x * q2.r.y) - (r.y * q2.r.x) + (r.z * q2.r.w));
            return result;
        }

        Quaternion operator*(const float s){
            Quaternion result;
            result.r.w = (r.w*s);
            result.r.x = (r.x*s);
            result.r.y = (r.y*s);
            result.r.z = (r.z*s);
            return result;
        }

        operator Quatstruct (void) {
            return r;   
        }





        
};

Quaternion structtoquat(Quatstruct qstruct){
    Quaternion result;
    result.r = qstruct;
    return result;
}

Quatstruct quattostruct(Quaternion qquat){
    Quatstruct result;
    result = qquat.r;
    return result;
}

Quaternion axisangletoquat(float theta,Quaternion axis,bool radians){
    radians == false ? theta = theta * (M_PI/180) : theta = theta;
    
    theta = theta/2;
    axis = axis.normalize();
    //printf("axissize = %f\n",axis.magnitude());
    Quaternion q;
    q.r.w = cos(theta);
    q.r.x = axis.r.x*sin(theta);
    q.r.y = axis.r.y*sin(theta);
    q.r.z = axis.r.z*sin(theta);
    //printf("axiswxyz %f,%f,%f,%f\n",axis.r.w,axis.r.x,axis.r.y,axis.r.z);
    //printf("qwxyz %f,%f,%f,%f\n",q.r.w,q.r.x,q.r.y,q.r.z);
    q = q.normalize();
    
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
    Quaternion q = axisangletoquat(_theta,axis,false); // construct the rotation quaternion from the input axis and theta values

    Quaternion _q = inv(q);
    result = (q*torotate)*_q;// rotate the original quaternion by the rotatinon quaternion
    return result;
}

Quatstruct intergrategyros(Quatstruct prevstate,Vector3float gyromess,float deltatime){
    Quaternion gyromesquat;
    Quatstruct result;
    //printf("%f,%fi,%fj,%fk\n",prevstate.w,prevstate.x,prevstate.y,prevstate.z);

    float degreestoradian = M_PI/180;

    gyromesquat.r.x = gyromess.x * degreestoradian;
    gyromesquat.r.y = gyromess.y * degreestoradian;
    gyromesquat.r.z = gyromess.z * degreestoradian;

    float gyromag = gyromesquat.magnitude();

    gyromesquat = gyromesquat.normalize();

    printf("gyromesquat %f,%fi,%fj,%fk\n",gyromesquat.r.w,gyromesquat.r.x,gyromesquat.r.y,gyromesquat.r.z);

    Quaternion qdelta = axisangletoquat(deltatime*gyromag, gyromesquat, true);

    printf("qdelta %f,%fi,%fj,%fk\n",qdelta.r.w,qdelta.r.x,qdelta.r.y,qdelta.r.z);
    
    Quaternion tomult = structtoquat(prevstate);

    result = tomult * qdelta;
    return result;
}

Vector3float Quattoeuler(Quatstruct q){
    Vector3float result;
    result.x = atan2(2*(q.w*q.x + q.y*q.z), 1 - (2*(pow(q.x,2)+pow(q.y,2))));
    result.y = asin(2*(q.w*q.y - q.x*q.z));
    result.z = atan2(2*(q.w*q.z + q.x*q.y), 1 -(2*(pow(q.y,2)+pow(q.z,2))));

    float radiantodegrees = 180/3.14159;

    result.x *= radiantodegrees;
    result.y *= radiantodegrees;
    result.z *= radiantodegrees;
    return result;
}



#endif // quats
