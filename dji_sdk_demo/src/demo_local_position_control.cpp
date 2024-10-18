/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include <nav_msgs/Odometry.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher acelerationPub;
ros::Publisher odometryPub;
ros::Publisher controlPub;
ros::Publisher initPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Imu local_imu;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3Stamped local_velocity;
geometry_msgs::Twist control_velocity;
std_msgs::Float32 altura;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "MATRICE_100_ROS_OSDK");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber localVelocity = nh.subscribe("dji_sdk/velocity", 10, &local_velocity_callback);
  ros::Subscriber controlVelocity = nh.subscribe("m100/velocityControl", 10, &control_velocity_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber imuSub     = nh.subscribe("dji_sdk/imu", 10, &imu_callback);
  ros::Subscriber h_takeoff     = nh.subscribe("dji_sdk/height_above_takeoff", 10, &h_takeoff_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  controlPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);
  acelerationPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/linear_aceleration", 10);
  odometryPub = nh.advertise<nav_msgs::Odometry>("/dji_sdk/odometry", 10);
  initPub = nh.advertise<geometry_msgs::Twist>("/m100/velocityControl", 10);

   // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  set_local_position();

             //! Elimina velocidades residuales
    geometry_msgs::Twist initData;
    initData.linear.x=0;
    initData.linear.y=0;
    initData.linear.z=0;
    initData.angular.z=0;
    initPub.publish(initData);

  if(is_M100())
  {

    ROS_INFO("M100 taking off!");
    //takeoff_result=runmotors();
    takeoff_result = M100monitoredTakeoff();

  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if(takeoff_result)
  {

    //! Start Mission by setting Target state to 1
    target_set_state = 1;
    ROS_INFO("*****OSDK OK*****");
    
    ROS_INFO("Run Controller!!!");
  }

  ros::spin();
  return 0;
}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  local_position = *msg;
  double xCmd, yCmd, zCmd;
  //sensor_msgs::Joy controlPosYaw;
  //sensor_msgs::Joy controllerSof;
  
  // Down sampled to 50Hz loop
  if (elapsed_time > ros::Duration(0.01)) {
    start_time = ros::Time::now();
    
    //if (target_set_state == 1) {
    if (true) {
      //! First arbitrary target
      if (current_gps_health > 3) {
        
        double RollInRad = toEulerAngle(current_atti).x;
        double PichInRad = toEulerAngle(current_atti).y;
        double YawInRad = toEulerAngle(current_atti).z;
   
        sensor_msgs::Joy odometria;
        //odometria.header.frame_id = "world";
	      ///odometria.child_frame_id = "drone_link";
	      //odometria.header.stamp = ros::Time::now();
        //odometria.axes.push_back(RollInRad);
        //odometria.axes.push_back(PichInRad);
        //odometria.axes.push_back(YawInRad-(C_PI/2));
        //odometria.axes.push_back(local_imu.angular_velocity.x);
        //odometria.axes.push_back(local_imu.angular_velocity.y);
        //odometria.axes.push_back(local_imu.angular_velocity.z);
        odometria.axes.push_back(local_imu.linear_acceleration.x);
        odometria.axes.push_back(local_imu.linear_acceleration.y);
        odometria.axes.push_back(local_imu.linear_acceleration.z);
        //odometria.axes.push_back(local_velocity.vector.y);
        //odometria.axes.push_back(-local_velocity.vector.x);
        //odometria.axes.push_back(local_velocity.vector.z);
        //odometria.axes.push_back(local_position.point.y);
        //odometria.axes.push_back(-local_position.point.x);
        //odometria.axes.push_back(altura.data);
        acelerationPub.publish(odometria);

	      nav_msgs::Odometry dron_odometry;
	      dron_odometry.header.frame_id = "world";
	      dron_odometry.child_frame_id = "drone_link";
	      dron_odometry.header.stamp = ros::Time::now();
	      dron_odometry.pose.pose.orientation.x = current_atti.x;
	      dron_odometry.pose.pose.orientation.y = current_atti.y;
	      dron_odometry.pose.pose.orientation.z = current_atti.z;
	      dron_odometry.pose.pose.orientation.w = current_atti.w;
  	    dron_odometry.pose.pose.position.x = local_position.point.x;
	      dron_odometry.pose.pose.position.y = local_position.point.y;
	      dron_odometry.pose.pose.position.z = altura.data;
	      dron_odometry.twist.twist.linear.x = local_velocity.vector.x;
	      dron_odometry.twist.twist.linear.y = local_velocity.vector.y;
	      dron_odometry.twist.twist.linear.z = local_velocity.vector.z;
	      dron_odometry.twist.twist.angular.x = local_imu.angular_velocity.x;
	      dron_odometry.twist.twist.angular.y = local_imu.angular_velocity.y;
	      dron_odometry.twist.twist.angular.z = local_imu.angular_velocity.z;

	      odometryPub.publish(dron_odometry);
	
	


        uint8_t flag = (0x40 | 0x00 | 0x08 | 0x02 |  0x01 );
        //uint8_t flag = (0x00 | 0x00 | 0x00 | 0x02 | 0x01 );
        sensor_msgs::Joy controllerPC;
        controllerPC.axes.push_back(control_velocity.linear.x);
        controllerPC.axes.push_back(control_velocity.linear.y);
        controllerPC.axes.push_back(control_velocity.linear.z);
        controllerPC.axes.push_back(control_velocity.angular.z);
        controllerPC.axes.push_back(flag);
        controlPub.publish(controllerPC);  

        //ROS_INFO("x=%f, y=%f, z=%f, yaw=%f ...", local_position.point.x,local_position.point.y, local_position.point.z,yawInRad );
        //ROS_INFO("ul=%f, um%f, un%f, w%f ...", local_velocity.vector.x,local_velocity.vector.y, local_velocity.vector.z, local_velocity.vector.z-local_velocity.vector.z);
  
      }
      else
      {
        ROS_INFO("Cannot execute Local Position Control");
        ROS_INFO("Not enough GPS Satellites");
        //! Set Target set state to 0 in order to stop Local position control mission
        target_set_state = 0;
      }
    }
  }
}

void local_ctrl(float &xCmd, float &yCmd, float &zCmd, float &psiCmd)
{


}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  local_velocity = *msg;
}

void control_velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  control_velocity = *msg;
}

void h_takeoff_callback(const std_msgs::Float32::ConstPtr& msg)
{
  altura = *msg;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
 local_imu = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */

bool
runmotors(){
	
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }
  return true;
}
	
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
}

