
#include <ros.h>
#include <std_msgs/Int16.h>
#include <Wire.h>

ros::NodeHandle  nh; 
std_msgs::Int16 ldrsol;
std_msgs::Int16 ldrsag;
ros::Publisher ldrright_pub("ldrright", &ldrsol);
ros::Publisher ldrleft_pub("ldrleft", &ldrsag);


int ldr1_i = A1;
int ldr2_i = A0;
int ldr1 = 0;
int ldr2 = 0;


float temp=0;
float temp1=0;

float rps=0;
float rps1=0;


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(ldr1_i, OUTPUT);
  pinMode(ldr2_i, OUTPUT);

  
  nh.initNode();
  nh.advertise(ldrright_pub );
  nh.advertise(ldrleft_pub );

 
}

void loop()
{
ldr1 = analogRead(ldr1_i);
ldr2 = analogRead(ldr2_i);
  ldrsag.data = ldr1;
   ldrsol.data = ldr2;

ldrright_pub.publish(&ldrsol);
ldrleft_pub.publish(&ldrsag);
nh.spinOnce();


}
  
