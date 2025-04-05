#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <stdio.h>

#include <enoga/enoga.h>

#include "std_msgs/builtin_int8.h"

geometry_msgs::TwistStamped vs;
geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped ps;
geographic_msgs::GeoPoseStamped gps;

int id=4;

double uavRollENU, uavPitchENU, uavYawENU;
bool isSendNextVelocity;

int rosInfoTimeDealy;

mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

ros::Publisher local_pos_pub;

void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

double x = 0;
double y = 5;
double z = 2;
double rot_x = 0;
double rot_y = 0;
double rot_w = 0;
double rot_z = 0;
double yaw = 0;
double leader_x = 0, leader_y = 0, leader_z = 0;

void leadposeCallback(
    const geometry_msgs::PoseStampedPtr &msg) // Topic messages callback
{
  leader_x = msg->pose.position.x;
  leader_y = msg->pose.position.y;
  leader_z = msg->pose.position.z;
}

void up() {
  ROS_WARN_STREAM("up");
  z += 1;
  isSendNextVelocity = true;
}

void zero_z() {
  ROS_WARN_STREAM("z is zero");
  z = 0.0;
  isSendNextVelocity = true;
}
void down() {
  ROS_WARN_STREAM("down");
  z -= 1;
  isSendNextVelocity = true;
}

void left() {
  ROS_WARN_STREAM("left");
  y -= 1;
  isSendNextVelocity = true;
}

void right() {
  ROS_WARN_STREAM("right");
  y += 1;
  isSendNextVelocity = true;
}

void forward() {
  ROS_WARN_STREAM("forward");
  x += 1;
  isSendNextVelocity = true;
}

void xy_to_zero() {
  ROS_WARN_STREAM("set XY to zero");
  x = 0.0;
  y = 0.0;
  x = 0.0;
  isSendNextVelocity = true;
}

void backward() {
  ROS_WARN_STREAM("backward");
  x -= 1;
  isSendNextVelocity = true;
}

void turn_clockwise() {
  ROS_WARN_STREAM("turn clockwise");
  rot_z += 0.1;
  isSendNextVelocity = true;
}

void turn_anticlockwise() {
  ROS_WARN_STREAM("turn anti-clockwise");
  rot_z -= 0.1;
  isSendNextVelocity = true;
}

void OFFBOARD() {
  offb_set_mode.request.custom_mode = "OFFBOARD";
  set_mode_client.call(offb_set_mode);

  if (offb_set_mode.response.mode_sent)
    ROS_WARN_STREAM("Offboard enabled");
}

void AUTO_RTL() {
  offb_set_mode.request.custom_mode = "AUTO.RTL";
  set_mode_client.call(offb_set_mode);

  if (offb_set_mode.response.mode_sent)
    ROS_WARN_STREAM("AUTO.RTL enabled");
}

void AUTO_LAND() {
  offb_set_mode.request.custom_mode = "AUTO.LAND";
  set_mode_client.call(offb_set_mode);

  if (offb_set_mode.response.mode_sent)
    ROS_WARN_STREAM("AUTO.LAND enabled");
}

void arm_vehicle() {
  arm_cmd.request.value = true;
  arming_client.call(arm_cmd);
  if (arm_cmd.response.success)
    ROS_WARN_STREAM("Vehicle armed");
}

void disarm_vehicle() {
  arm_cmd.request.value = false;
  arming_client.call(arm_cmd);
  if (arm_cmd.response.success)
    ROS_WARN_STREAM("Vehicle disarmed");
}

void follow() {
  x = int(leader_x) + 1;
  y = int(leader_y);
  z = int(leader_z);
  isSendNextVelocity = true;
}

enoga::enoga enoga_command;
void handle_form(const enoga::enoga &enoga_value) {
	enoga_command=enoga_value;
	if(enoga_command.id==id) {
		//пока что полагаем, что первый дрон построения и дрон-лидер - один и тот же дрон
		/*x+=enoga_command.location.position.x;
		y+=enoga_command.location.position.y;
		z+=enoga_command.location.position.z;*/
		x=leader_x+enoga_command.location.position.x;
		y=leader_y+enoga_command.location.position.y;
		z=leader_z+enoga_command.location.position.z;
		isSendNextVelocity=true;
	}
}

std_msgs::Int8 command;
void handle_commander(const std_msgs::Int8 &command_value) {
  command = command_value;
  switch (command.data) {
  case 1: {
    arm_vehicle();
    break;
  }
  case 2: {
    OFFBOARD();
    break;
  }
  case 3: {
    up();
    break;
  }
  case 4: {
    down();
    break;
  }
  case 5: {
    left();
    break;
  }
  case 6: {
    right();
    break;
  }
  case 7: {
    forward();
    break;
  }
  case 8: {
    backward();
    break;
  }
  case 9: {
    turn_clockwise();
    break;
  }
  case -1: {
    AUTO_RTL();
    break;
  }
  case -2: {
    AUTO_LAND();
    break;
  }
  case -5: {
    OFFBOARD();
    arm_vehicle();
    // ros::Duration(3.0).sleep();
    // OFFBOARD();
    // ros::Duration(3.0).sleep();
    // follow();
    break;
  }
  case -6: {
    AUTO_LAND();
    break;
  }
  case -7: {
    follow();
    break;
  }
  case -8: {
  	//formation
  }
  default: {
  }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "follower4");
  ros::NodeHandle nodeHandle;
  ros::Subscriber state_sub = nodeHandle.subscribe<mavros_msgs::State>(
      "/uav4/mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub =
      nodeHandle.advertise<geometry_msgs::PoseStamped>(
          "/uav4/mavros/setpoint_position/local", 10);
  ros::Subscriber sub = nodeHandle.subscribe("/uav0/mavros/local_position/pose",
                                             10, leadposeCallback);

  ros::Subscriber command_sub =
      nodeHandle.subscribe("/commands_to_follower", 1000, handle_commander);
  
  ros::Subscriber enoga_sub=nodeHandle.subscribe("/enoga_controller", 10, handle_form);

  isSendNextVelocity = false;

  geometry_msgs::PoseStamped pose;
  // pose.pose.position.x = 0;
  // pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  rosInfoTimeDealy = 0;

  // offb_set_mode.request.custom_mode = "OFFBOARD";
  // arm_cmd.request.value = true;
  //  ros::Subscriber commandSubscriber =
  //  nodeHandle.subscribe("/keyboard/keydown", 1, sendCommand);

  arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>(
      "/uav4/mavros/cmd/arming");
  set_mode_client =
      nodeHandle.serviceClient<mavros_msgs::SetMode>("/uav4/mavros/set_mode");

  local_pos_pub.publish(pose);

  // Wait for FCU connection.
  while (ros::ok()) {

    if (isSendNextVelocity == true) {
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.z = rot_z;

      local_pos_pub.publish(pose);
    }

    local_pos_pub.publish(pose);
    ros::spinOnce();
    //  loop_rate.sleep();
    // local_pos_pub.publish(pose);
  }
}
