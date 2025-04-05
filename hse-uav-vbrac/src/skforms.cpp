#include "iostream"
#include "ros/ros.h"
#include "std_msgs/builtin_int8.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <enoga/enoga.h>
#include <algorithm>
#include <formation.h>
#include <enoga/formationMsg.h>

//основная функция данной ноды для управления дронами
int main(int argc, char **argv) {
  ros::init(argc, argv, "commands_controller");
  ros::NodeHandle nh;
  ros::Rate rate(0.1);

  ros::Publisher commander_pub=nh.advertise<std_msgs::Int8>("/commands_controller", 10);
  std_msgs::Int8 command;
  
  //TODO: move to GUI and etc
  ros::Publisher formPub=nh.advertise<enoga::formationMsg>("/formation", 10);
  enoga::formationMsg formCmd;
  formCmd.form=-1;
  formCmd.parameter=5;
  formCmd.secangle=90;
  formCmd.arclength=120;

  int flag=1;
  while(ros::ok()) {
 	if(flag==2) {
  		command.data = 1; //start
  		commander_pub.publish(command);
  		ROS_INFO("Started");
  	} else if(flag==3) {
  		command.data = 2;//fly
  		commander_pub.publish(command);
  	} else if(flag==4) {
  		command.data = -5; //connect
  		commander_pub.publish(command);
  	}
  	if(flag==5) {
  	  	formCmd.form=0; //klin
  	}
  flag+=1;
  /*  } else if(!str.compare("formate1")) { //klin with first
    	ROS_INFO("start making klin with first");
    	formCmd.form=1;
    } else if(!str.compare("formate2")) { //snake
    	ROS_INFO("start making snake"); 
    	formCmd.form=2;
    } else if(!str.compare("formate3")) { //front
    	ROS_INFO("start making front"); 
    	formCmd.form=3;
    } else if(!str.compare("formate4")) { //column
    	ROS_INFO("start making column"); 
    	formCmd.form=4;
    } else if(!str.compare("formate5")) { //peleng
    	ROS_INFO("start making peleng"); 
    	formCmd.form=5;
    } else if(!str.compare("formate6")) { // flangs
    	ROS_INFO("start making flangs"); 
    	formCmd.form=6;
    } else if(!str.compare("formate7")) { // chess
    	ROS_INFO("start making chess"); 
    	formCmd.form=7;
    } else if(!str.compare("formate8")) { //rect
    	ROS_INFO("start making rect"); 
    	formCmd.form=8;
    } else if(!str.compare("formate9")) { //rect_empty
    	ROS_INFO("start making rect_empty"); 
    	formCmd.form=9;
    } else if(!str.compare("formate10")) { // rhombus
    	ROS_INFO("start making rhombus"); 
    	formCmd.form=10;
    } else if(!str.compare("formate11")) { //rhombus_empty
    	ROS_INFO("start making rhombus_empty"); 
    	formCmd.form=11;
    } else if(!str.compare("formate12")) { //sector
    	ROS_INFO("start making sector"); 
    	formCmd.form=12;
    } else if(!str.compare("formate13")) { //circle
    	ROS_INFO("start making circle");
    	formCmd.form=13;
    } */

    if(formCmd.form!=-1) {
	formPub.publish(formCmd);
	//break;
    }
    formCmd.form=-1;
//    ROS_INFO("Sended");
//    commander_pub.publish(command);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
