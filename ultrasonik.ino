#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound_range", &range_msg);
char frameid[] = "/ultrasound";

const int pingPin = 9;
const int echo = 10;
const boolean CENTIMETERS = true;
const boolean INCHES = false;

void setup() {
  
  nh.initNode();
  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  
  range_msg.min_range = 0.002;  
  range_msg.max_range = 0.150;  

  
  //Serial.begin(9600);
}

long getRange(int pinNumber, boolean in_centimeters){

    
      long duration, distance, inches, cm;
    
      
      pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);
    
      
      pinMode(pingPin, INPUT);
      duration = pulseIn(echo, HIGH);
    
      
      inches = microsecondsToInches(duration);
      cm = microsecondsToCentimeters(duration);

      if (in_centimeters) 
         distance = cm;
      else 
        distance = inches;
      //Serial.print(inches);
      //Serial.print("in, ");
      Serial.print(cm);
      Serial.print("cm");
      //Serial.println();

      return distance;

  
}

void loop() {
  range_msg.range=getRange(pingPin, CENTIMETERS);
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  nh.spinOnce();
  delay(500);
}

long microsecondsToInches(long microseconds) {
  
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  
  return microseconds / 29 / 2;
}
