#include "quats.h"
#include "stdio.h"



Quaternion q(1,1,1,1);

Quaternion q1(0,1,0,0);

Quaternion rotateq(0,0,1,0);

Quaternion qinv = inv(q);
Quaternion qcon = q.conjugate();

Quaternion q1rot = rotate(q1,rotateq,90);

int main(){
    printf("w%f,i%f,j%f,k%f\n",qcon.w,qcon.x,qcon.y,qcon.z);
    printf("w%f,i%f,j%f,k%f\n",qinv.w,qinv.x,qinv.y,qinv.z);
    printf("w%f,i%f,j%f,k%f\n",q1rot.w,q1rot.x,q1rot.y,q1rot.z);

}