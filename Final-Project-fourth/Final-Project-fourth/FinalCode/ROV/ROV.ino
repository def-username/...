#include "library.h"

EthernetUDP udp;

CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.
PIDController FPID,SPID;
double ki , kd , kp;
double sp,sp2;
double sensorValue;

float accX = 0 , accY = 0 , accZ = 0 , temp = 27;
unsigned int pressure = 1000 , depth = 0;



void send(float ccX , float accY , float temp , float pressure ,float depth);

void setup() {

  FPID.begin();
  SPID.begin();
  Serial.begin(9600);
  IMU_begin();
  PR_begin();
  FPID.tune(10, 20,20);
  SPID.tune(10,20,20);
  FPID.limit(0, 255);
  SPID.limit(0,255);
  FPID.setpoint(sp);
  SPID.setpoint(sp2);

  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05}; //0::1::2::3::4::5
  Ethernet.begin(mac,IPAddress(192,168,0,6));//192.168.0.6
  udp.begin(5000); //Port 5000
}
void loop() {
  bool Noforces = true;

    // Usb.Task();                                                    //Use to read joystick input to controller
    // JoyEvents.PrintValues();                                       //Returns joystick values to user
    // JoyEvents.GetValues( &fx, &fy, &Hat, &Twist, &slider, &Button);   //Copies joystick values to user

      uint8_t forces[5];

      int framesize = udp.parsePacket();
      if(framesize>0){
      do{
          int res= udp.read(forces,framesize+1);
        }
        while ((framesize = udp.available())>0);
        udp.flush(); 
      }
      ///Now We apply forces
      /**
       * 
       * 
       * Esraaaa
       * If forces is found make noForces false
       * */
      //PID
    forces();
    if(Noforces){
    //Inputs of PID:
    /**
     * X: accX
     * Y: accY
     * depth: Depth of ROV
     * */

//     Serial.print("X is ");
//     Serial.print(accX);
//     Serial.print("\tY is ");
//     Serial.print(accY);
//     Serial.print("\tZ is ");
//     Serial.print(accZ);
//     Serial.print("\t Pressure is ");
//     Serial.print(pressure);
//     Serial.print("\tDepth is ");
//     Serial.println(depth);      
//     delay(2000);

    double _Fx = FPID.compute(accX);
    double _Fy = FPID.compute(accY);
    double _Fz=SPID.compute(depth);

  }

///////////////////////////////
    IMU_readValues();//Read sensor values

    PR_get_Results(&pressure , 0);//We don't need temperature
    depth = PR_get_depth(pressure);

    //IMU must be read at any time as it is displayed in GUI
     accX = IMU_getValue(GET_X);
     accY = IMU_getValue(GET_Y);
     accZ = IMU_getValue(GET_Z);
     temp = IMU_getValue(GET_TEMP); 
     //Now we begin sending

     send(accX , accY , temp , pressure , depth);
   
}

//   //force(); 
//   if(Button==3){
//       motor.setSpeed(255);  // Run up at full speed.
//       delay(1000);
// }
//   else if(Button==4){
//      motor.setSpeed(-255);  // Run down at full speed.
//      delay(1000);
//   }
//   else{
//      motor.setSpeed(0);    // Stop.
//      delay(1000);
//     }

//   else 
//   {
//     force();
    
//   }
    


//-----------------------------------------------------------------------------------------------------
//void force(){
//  int row=3;
//  int col=1;
//  float td[row][col]={{fx},
//               {fy},
//               {M}
//};
//    float q[3][3]={        
//    {1.4142,0,0},
//    {0,1.4142,0},
//    {0,0,0}
//    };
//    float T[3][1] = {
//    {0},  
//    {0},  
//    {0}
//    };
//
//    for(int i=0;i<3;i++){
//        for(int j=0;j<col;j++){
//        for(int a=0; a<1;a++){
//            T[i][j] +=q[i][a] * td[a][j];
//        }
//        }
//    }
//
//    float Q[4][1]={
//    {0},
//    {T[0][0]},
//    {T[1][0]},
//    {T[2][0]}
//    };
//    float w[4][4]={{1,1,-1,-1},
//    {1,1,1,1},
//    {1,1,-1,-1},
//    {1,1,1,1}
//    };
//    float f[4][1]= {
//    {0},  
//    {0},  
//    {0},
//    {0}
//    };
//    for(int i=0;i<3;i++){
//        for(int j=0;j<col;j++){
//        for(int a=0; a<1;a++){
//            f[i][j] +=0.25* w[i][a] * Q[a][j];
//        }
//        }
//    }
//    Serial.print(f[4][0],DEC);
//
//    int FX=0;
//    int FY=0;
//    for(int i;i<4;i++){
//    FX +=f[i][0]*0.707106;
//    }
//    Serial.print(FX,DEC);
//    FY=(-f[0][0]+f[1][0]-f[2][0]+f[3][0])*0.707106;
//    Serial.print(FY,DEC);
//    return f,FX,FY;
//}
void forces()
{
  int i,j,k;
    for(i=0;i<first_row;i++)
    {
        for(j=0;j<second_col;j++)
        {
        mul[i][j]=0;
        for(k=0;k<first_col;k++)
        {
        mul[i][j]=mul[i][j]+matrix1[i][k]*matrix2[k][j];
        }
        }
    }
    for(i=0;i<first_row;i++)
    {
      for(j=0;j<second_col;j++)
      {
        Serial.println(mul[i][j]);
      }
    }
    delay(1000);
}


void send(float ccX , float accY , float temp , float pressure ,float depth){
  udp.beginPacket(udp.remoteIP() , udp.remotePort());
  //Send a variable
  udp.endPacket();
  ///Loop is iterated.
}
