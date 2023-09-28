#include "quats.h"
#include "stdio.h"
/*
Quaternion q(1,1,1,1);

Quaternion q1(1,0,0,0)  ;

Quaternion rotatezaxis(0,0,0,1);
Quaternion rotatexaxis(0,1,0,0);

Quaternion qinv = inv(q);
Quaternion qcon = q.conjugate();

Quaternion q1rot = rotate(q1,rotatezaxis,45);

Vector3float gyromes = {0,45,0};

Quaternion q2gyro = intergrategyros(q1,gyromes,1);

void printquat(Quaternion qtoprint){
    printf("%f,%fi,%fj,%fk\n",qtoprint.w,qtoprint.x,qtoprint.y,qtoprint.z);
}


int main(){
    printquat(axisangletoquat(45,q1));
    printquat(qcon);
    printquat(qinv);
    printquat(q1rot);
    q1rot = rotate(q1rot,rotatexaxis,45);
    printquat(q1rot);
    q1rot = rotate(q1rot,rotatexaxis,-45);
    printquat(q1rot);

    printquat(q2gyro);
}
*/