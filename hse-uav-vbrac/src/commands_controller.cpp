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
#include <geometry_msgs/Point.h>
#include <enoga/points.h>
#include <vector>

struct Point {
	float x;
	float y;
};

//основная функция данной ноды для управления дронами
int main(int argc, char **argv) {
  ros::init(argc, argv, "commands_controller");
  ros::NodeHandle nh;

  ros::Publisher commander_pub=nh.advertise<std_msgs::Int8>("/commands_controller", 10);
  std_msgs::Int8 command;
  
  //TODO: move to GUI and etc
  ros::Publisher formPub=nh.advertise<enoga::formationMsg>("/formation", 10);
  ros::Publisher waypoints_pub=nh.advertise<enoga::points>("/coordinates_for_leader", 10);
  enoga::points points_msg;
  std::vector<Point> points;
  Point one;
  Point two;
  Point three;
  Point four;
  Point five;
  Point six;
  Point sev;
  Point eig;
  Point nin;
  Point ten;
  Point ele;
  Point twe;
  Point thi;

  one.x=5;
  one.y=0;
  two.x=4.33;
  two.y=2.5;
  three.x=3.535;
  three.y=3.535;
  four.x=5;
  four.y=4.33;
  five.x=0;
  five.y=5;
  six.x=-2.5;
  six.y=4.33;
  sev.x=-3.535;
  sev.y=3.535;
  eig.x=-4.33;
  eig.y=2.5;
  nin.x=-5;
  nin.y=0;
  ten.x=-4.33;
  ten.y=-2.5;
  ele.x=-3.535;
  ele.y=-3.535;
  twe.x=-2.5;
  twe.y=-4.33;
  thi.x=0;
  thi.y=-5;
  points.push_back(one);
  points.push_back(two);
  points.push_back(three);
  points.push_back(four);
  points.push_back(five);
  points.push_back(six);
  points.push_back(sev);
  points.push_back(nin);
  points.push_back(ten);
  points.push_back(ele);
  points.push_back(twe);
  points.push_back(thi);


  for(std::vector<Point>::iterator it=points.begin(); it!=points.end(); ++it) {
  	geometry_msgs::Point point;
  	point.x=(*it).x;
  	point.y=(*it).y;
  	point.z=2;
  	points_msg.points.push_back(point);
  }
  
  enoga::formationMsg formCmd;
  formCmd.form=-1;
  formCmd.parameter=5;
  formCmd.secangle=90;
  formCmd.arclength=120;
  
    std::cout << "commands:\n\tstart\n\tfly\n\tup\n\tdown\n\tleft\n\tright\n\tforward\n\tbackward\n\tauto.rtl\n\tauto.land\n\tconnect\n\tfollow\n\tdisconnect\n\tformate - klin\n\tformate1 - klinWithFirst\n\tformate2 - snake\n\tformate3 - front\n\tformate4 - column\n\tformate5 - peleng\n\tformate6 - flangs\n\tformate7 - chess\n\tformate8 - rect\n\tformate9 - rect_empty\n\tformate10 - rhombus\n\tformate11 - rhombus_empty\n\tformate12 - sector\n\tformate13 - circle\n\trotation - clockwise\n\trotation1 - contrclockwise\n\texit\n\n";

  std::string str;

  while (true) {
    std::cin >> str;
    if (!str.compare("start")) {
      ROS_INFO("start");
      command.data = 1;
      //  isflightStarted = true;
    } else if (!str.compare("fly")) {
      ROS_INFO("fly");
      command.data = 2;
    } else if (!str.compare("up")) {
      ROS_INFO("up");
      command.data = 3;
    } else if (!str.compare("down")) {
      ROS_INFO("down");
      command.data = 4;
    } else if (!str.compare("left")) {
      ROS_INFO("left");
      command.data = 5;
    } else if (!str.compare("right")) {
      ROS_INFO("right");
      command.data = 6;
    } else if (!str.compare("forward")) {
      ROS_INFO("forward");
      command.data = 7;
    } else if (!str.compare("backward")) {
      ROS_INFO("backward");
      command.data = 8;
    } else if (!str.compare("rotation")) {
      //TO DO
      ROS_INFO("rotation clockwise");
      command.data = 9;
    } else if(!str.compare("rotation1")) {
      ROS_INFO("rotation contrclockwise");
      command.data=10;
    } else if (!str.compare("auto.rtl")) {
      ROS_INFO("auto.rtl");
      command.data = -1;
    } else if (!str.compare("auto.land")) {
      ROS_INFO("auto.land");
      command.data = -2;
    } else if (!str.compare("connect")) {
      ROS_INFO("Follower is connected");
      command.data = -5;
    } else if (!str.compare("disconnect")) {
      ROS_INFO("Follower is disconnected");
      command.data = -6;
    } else if (!str.compare("follow")) {
      ROS_INFO("follow to leader");
      command.data = -7;
    } else if(!str.compare("way")) {
    	ROS_INFO("way started");
    	command.data=-8;
    } else if(!str.compare("formate")) { //klin
    	ROS_INFO("start making klin"); 
    	formCmd.form=0;
    } else if(!str.compare("formate1")) { //klin with first
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
    } else if (!str.compare("exit")) {
      break;
    }
	if(command.data==-8) {
	  waypoints_pub.publish(points_msg);
  }
	if(formCmd.form!=-1)
    	formPub.publish(formCmd);
    	formCmd.form=-1;
    commander_pub.publish(command);

    ros::spinOnce();
  }

  return 0;
}
