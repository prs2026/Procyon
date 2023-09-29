#include "quats.h"
#include "stdio.h"

Quaternion q(1,1,1,1);

Quaternion q1(0,1,0,0)  ;

Quaternion xaxis(0,1,0,0);
Quaternion yaxis(0,0,1,0);
Quaternion zaxis(0,0,0,1);


Quaternion qinv = inv(q);
Quaternion qcon = q.conjugate();

Quaternion q1rot = rotate(q1,zaxis,45);

const Vector3float gyromes = {20,20,0};

//Quaternion q2gyro = intergrategyros(q1,gyromes,1);

Quaternion qtorot(1,0,0,0);


void printquat(Quaternion qtoprint){
    printf("%f,%fi,%fj,%fk\n",qtoprint.r.w,qtoprint.r.x,qtoprint.r.y,qtoprint.r.z);
}


int main(){

    qtorot = intergrategyros(qtorot,gyromes,1);

    printquat(qtorot);


    //qtorot = intergrategyros(qtorot,gyromes,1);

    printquat(qtorot);

    Vector3float euler;

    euler = Quattoeuler(qtorot.r);

    printf("%f,%f,%f\n",euler.x,euler.y,euler.z);


    //printquat(q2gyro);
}
