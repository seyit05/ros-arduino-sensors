#include <MadgwickAHRS.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <LSM6.h>
#include <LIS3MDL.h>
 
// Set up the ros node and publishers
ros::NodeHandle nh;
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("magnet", &msg_magnet);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("gyro", &msg_gyro);
unsigned long pubTimer = 0;


LIS3MDL mag;
LSM6 imu;
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;



 
void setup()
{
 
    nh.initNode();
    nh.advertise(pub_accel);
    nh.advertise(pub_magnet);
    nh.advertise(pub_gyro);
 
    // Wait until connected
    while (!nh.connected())
        nh.spinOnce();
    nh.loginfo("ROS startup complete");
 
    Wire.begin();
 
    
    if (!mag.init())
    {
        nh.logerror("Failed to autodetect compass type!");
        
    }
    mag.enableDefault();


  // start the IMU and filter
  //CurieIMU.begin();
  //CurieIMU.setGyroRate(0);
  //CurieIMU.setAccelerometerRate();
  filter.begin(25);

  // Set the accelerometer range to 2G
  //CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  //CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  // This is to maximize imu performance
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();


 
    if (!imu.init())
    {
        nh.logerror("Failed to autodetect gyro type!");
    }
    imu.enableDefault();



  // start the IMU and filter
  //CurieIMU.begin();
  //CurieIMU.setGyroRate(25);
  //CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  //CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  //CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  // This is to maximize imu performance
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
 
    pubTimer = millis();
}
 
void loop()
{
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long microsNow;
    if (millis() > pubTimer)

    if (microsNow - microsPrevious >= microsPerReading) 
    {
      
    
    {
        mag.read();
        imu.read();

        
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    filter.updateIMU(gx, gy, gz, ax, ay, az);



        // Compass - accelerometer:
        // 16-bit, default range +-2 g, sensitivity 0.061 mg/digit
        // 1 g = 9.80665 m/s/s
        // e.g. value for z axis in m/s/s will be: compass.a.z * 0.061 / 1000.0 * 9.80665
        //      value for z axis in g will be: compass.a.z * 0.061 / 1000.0
        // Gravity is measured as an upward acceleration:
        // Stationary accel. shows +1 g value on axis facing directly "upwards"
        // Convert values to g
        msg_accel.x = (float)(imu.a.x)*0.061/1000.0;
        msg_accel.y = (float)(imu.a.y)*0.061/1000.0;
        msg_accel.z = (float)(imu.a.z)*0.061/1000.0;
       
 
        // Compass - magnetometer:
        // 16-bit, default range +-2 gauss, sensitivity 0.080 mgauss/digit
        msg_magnet.x = mag.m.x;
        

        msg_magnet.y = mag.m.y;
        

        msg_magnet.z = mag.m.z;
        
        
        // Gyro:
        // 16-bit, default range +-245 dps (deg/sec), sensitivity 8.75 mdps/digit
        // Convert values to rads/sec
        msg_gyro.x = (float)(imu.g.x)*0.00875*M_PI/180.0;
        
        msg_gyro.y = (float)(imu.g.y)*0.00875*M_PI/180.0;
        
        msg_gyro.z = (float)(imu.g.z)*0.00875*M_PI/180.0;

    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
 
        pub_accel.publish(&msg_accel);
        pub_magnet.publish(&msg_magnet);
        pub_gyro.publish(&msg_gyro);

        microsPrevious = microsPrevious + microsPerReading;
    }  
        pubTimer = millis() + 10;  // wait at least 10 msecs between publishing
    }
 
    nh.spinOnce();
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
