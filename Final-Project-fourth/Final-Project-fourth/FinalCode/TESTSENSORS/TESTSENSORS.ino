#include "library.h"

void setup(){
    Serial.begin(9600);
    IMU_begin();
    PR_begin();
}

void loop(){
    float x = -1 , y = -1 ;
    int pressure , depth;
    IMU_readValues();
    x = IMU_getValue(GET_X);
    y = IMU_getValue(GET_Y);
    PR_get_Results(&pressure , 0);
    Serial.print("X : ");
    Serial.print(x);
    Serial.print("\tY : ");
    Serial.print(y);
    Serial.print("\tPressure : ");
    Serial.print(pressure);
    Serial.print("Depth : ");
    Serial.println(PR_get_depth(pressure));
    delay(1000);
    
    }
