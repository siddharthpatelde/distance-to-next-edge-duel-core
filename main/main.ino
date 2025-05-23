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
#define angle_upper_bound 355

/*
define the constnat height of the lidar, to calculate theoretical
distance (Hypotenuse from right angle triangle)
*/

#define scan_height 22 //in cm

/*
define the tolerance for edge detecation logic.
i.e when distnace comming from sensor is "k" times bigger than the one we
calculated from theory (triangle theory), then we say that we have edge detection at that angle
here "k""is our tolerance factor.
*/

#define tolerance_low 1.2  // 20% increse in theoretical value
#define tolerance_high  1.5 // 50% increse in theoretical value

/*
i have definded tolerance factors, so that i can ignore some values after first edge detection
*/


#include <ArduinoJson.h> //to use json string as output

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//////////////////////////////////////////////////////[   core 0    ]///////////////////////////////////////////////////////// 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



void setup(){ 
  //for the PICO to print the stuff in serial monitor
  Serial.begin(115200);

  // bind the RPLIDAR driver to the arduino hardware serial
  Serial1.begin(115200);  // For RPLidar
  lidar.begin(Serial1);

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

    distance = lidar.getCurrentPoint().distance;          // in mm 
    angle = lidar.getCurrentPoint().angle;                // in Degree
    new_scan_flag = lidar.getCurrentPoint().startBit;     // 0 or 1



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //              data filtering 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /*
    here i write logic to find all the positive distnaces and 
    counting how many valid angles, distnaces value are we getting 
    in one scan
    */

    if(distance > 0 && angle >= angle_lower_bound && angle < angle_upper_bound){
      
      /* variable to store the distnace from next edge using defined function 
      [ float get_distance_to_next_edge(float angle_degrees, float distance_mm) ] */

      float distance_from_edge = get_distance_to_next_edge(angle,distance);

      // Serial.println(distance_from_edge); // Print newline for readability

      JsonDocument doc; 

      doc["scan height"] = scan_height;
      doc["distnace"] = distance_from_edge;

      serializeJson(doc, Serial);
      Serial.println(); // Print newline for readability

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
       analogWrite(RPLIDAR_MOTOR, 180); //150 pwm value of 5.5Hz case
       delay(1000);
    }
  }

}

float get_distance_to_next_edge(float angle_degrees, float distance_mm) {

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //                  step: 1 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //comnvert distnaces from mm to cm and angles fro degrees to radianse for trigonometry formulas

    float distance_cm = distance_mm / 10;                 // Convert distance in mm to cm  
    float angle_radianse = ((angle_degrees - 270) * PI) / 180;    // Concert angles in degrees to radianse, and applying quadrant calculations

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //                  step: 2 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //compute the desred or theoretical value fo distnace using the hight of the lidar that we set

    float distance_calculated = scan_height / cos(angle_radianse);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //                  step: 3 
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /*
    applying edge detection logic, we say we have edge detection 
    when mesured distnace is 20% bigger and 45% smaller then the one we calculated.
    */

    if(distance_cm > distance_calculated * tolerance_low){

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //                  step: 4 
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        // when edge is detected, finding the distance to the edge 
      float distance_to_next_edge = tan(angle_radianse) * scan_height;
        return distance_to_next_edge; // Print newline for readability
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//////////////////////////////////////////////////////[   core 1    ]///////////////////////////////////////////////////////// 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup1(){


}

void loop1(){

}

