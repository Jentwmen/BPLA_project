#include "iostream"
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <enoga/points.h>
#include <vector>

//size_t flag=0; //1 - poses were changed, 0 - poses weren't changed
//Структура - точка, нужна для формирования предварительного массива точек, чтобы обойтись одним вектором
struct Point {
	float x;
	float y;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_surf_pub");
  ros::NodeHandle nh;
  
  //Сюда будем отправлять массив из пограничных точек (только точки, в которых находятся БЛА!)
  ros::Publisher vbrac_surf_pub=nh.advertise<enoga::points>("/vbrac_surf_input", 10);
  
  //Ниже пример для 4-х точек, проверенный
  enoga::points points_msg;
  std::vector<Point> points;
  Point one;
  Point two;
  Point three;
  Point four;
  one.x=0;
  one.y=4;
  two.x=4;
  two.y=4;
  three.x=4;
  three.y=-4;
  four.x=0;
  four.y=-4;
  points.push_back(one);
  points.push_back(two);
  points.push_back(three);
  points.push_back(four);
  
  ROS_INFO("Pub msg:\n");
  
  //Формируем именно сообщение, в котором будет передан массив точек
  for(std::vector<Point>::iterator it=points.begin(); it!=points.end(); ++it) {
  	geometry_msgs::Point point;
  	point.x=(*it).x;
  	point.y=(*it).y;
  	point.z=0;
  	points_msg.points.push_back(point);
  	ROS_INFO("x - %f", (*it).x);
  	ROS_INFO("y - %f", (*it).y);
  }

  ros::Rate rate(20.0);
  //flag=0;
  //ros::Time last_request = ros::Time::now();
  //vbrac_surf_pub.publish(points_msg);
  //ROS_INFO("%d", flag);
  while (ros::ok()) {
    //if(flag==0&&(ros::Time::now() - last_request > ros::Duration(3.0))) {
    vbrac_surf_pub.publish(points_msg);
    	//flag=1;
    	//ROS_INFO("work");
    //}
    //last_request = ros::Time::now();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
