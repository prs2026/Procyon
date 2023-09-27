#include <cmath>
#include "quats.h"
#include <stdio.h>
#include <string.h>

Quaternion base(0,2,1,1);
Quaternion base2(0,0.5,0.25,0.25);


int main(){
    printquat(base2);
    printquat(base);
    base.normalize(); 
    printquat(base);
    
    printquat(base.tofloat());
    base2.normalize();

    printf("hello world\n");
    
    printquat(base2);
    Quaternion result = addquat(base,base2);

    printf("add result: \n");

    printquat(result);
    
    
    
    return 0;

}