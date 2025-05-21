#include <ArduinoJson.h>
#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.

int point_count = 0; //defining the point count variable to find how many mesurements are there in one scan 
int new_scan_flag = 0; // defining flag variable to show if it is new scan or not i.e 1 --> new scan start, 0 --> not a new scan
float distance = 0;
float angle = 0;

/*
defining the lower and upper boundry to filter the incomming data from lidar.
i.e. if lower bound is 270 and upper bound is 360 than we are only taking angles and 
distnaces value in range of [270,360]
*/

#define angle_lower_bound 270
#define angle_upper_bound 360





//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//////////////////////////////////////////////////////[   core 0    ]///////////////////////////////////////////////////////// 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



void setup(){
  //for the PICO to print the stuff in serial monitor
  Serial.begin(115200);

  
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial);


  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);

}
  
void loop(){
  if (IS_OK(lidar.waitPoint())) {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //              data retrieval
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /* 
    here we take the data from lidar like, distances in cm, 
    angles in mm and a flat value for if its new scan or not
    */ 

    distance = lidar.getCurrentPoint().distance;          // 30 cm minimum
    angle = lidar.getCurrentPoint().angle;                // 0-360 deg
    new_scan_flag = lidar.getCurrentPoint().startBit;     // 0 or 1



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //              data filtering 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /*
    here i write logic to find all the positive distnaces and 
    counting how many valid angles, distnaces value are we getting 
    in one scan
    */

    if(distance > 0 && angle >= angle_lower_bound && angle <= angle_upper_bound){



      if (new_scan_flag) {
        point_count = 0; //setting the point count again to zero when new scan starts i.e startBit == 1 
      }

      point_count++;
    }
    




  }else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       //detected...
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 150); //150 pwm value of 5.5Hz case
       delay(1000);
    }
  }

}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//////////////////////////////////////////////////////[   core 0    ]///////////////////////////////////////////////////////// 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup1(){


}

void loop1(){


}

