#ifndef UAV_H
#define UAV_H
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/builtin_int8.h>
#include <enoga/enoga.h>
#include <enoga/follower.h>
#include <enoga/points.h>
#include <map>
#include <string>
#include <formation.h>
#include <enoga/formationMsg.h>
#include <enoga/battery.h>
#include <enoga/guiMas.h>
#include <enoga/groups.h>
#include <rosgraph_msgs/Clock.h>
#include <mavros_msgs/HomePosition.h>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <math.h>
#include <enoga/algMsg.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>


class UAV {
	private:
	
		ros::NodeHandle ndh;
		std::vector<size_t> active_uavs;
		std::map<size_t, int> uavs; //по сути дублирует active_uavs, но был оставлен для обратной совместимости с реализованным методом проверки достижения необходимых позиций БЛА в алгоритме формаций
		std::vector<geometry_msgs::Point> surf_points;
		float per_save;
		enoga::follower last_follower_msg;
		enoga::guiMas datgui;
		enoga::groups groupsmsg;
		geometry_msgs::PoseStamped ps;
		mavros_msgs::SetMode offb_set_mode;
		mavros_msgs::State cur_state;
		mavros_msgs::CommandBool arm_cmd;
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::Publisher local_pos_pub;
		ros::Publisher battery_pub; //
		ros::Publisher commands_to_follower;
		ros::Publisher commands_to_leader; //отправка сообщений лидеру от последователей
		ros::Publisher enoga_pub;
		ros::Publisher vbrac_surf_inner_pub;
		ros::Publisher home_pub;
		ros::Publisher followers_msgs_pub;
		ros::Publisher goal_to_follower_pub;
		ros::Publisher vbrac_bounding_points;
		ros::Publisher guimas_pub;
		ros::Publisher groups_pub;
		
		std_msgs::Int8 command_to_follower;
		enoga::follower command_for_swarm; //последний код команды смены статуса члена роя с определенным id (0 - установлен новый лидер, 1 - установлен новый первый БЛА построения, 2 - отключен от роя (для тех, которые отправлены на посадку из-за малого заряда батареи), 3 - подключен к рою (для тех, которые запустили вновь))
		ros::Subscriber commandSub;
		ros::Subscriber guimas_sub;
		ros::Subscriber guiCallback_sub;
		ros::Subscriber stateSub;
		ros::Subscriber swarmSub;
		ros::Subscriber enoga_sub;
		ros::Subscriber followers_commands_sub;
		ros::Subscriber leaderPosSub;
		ros::Subscriber firstPosSub;
		ros::Subscriber commands_to_follower_sub;
		ros::Subscriber formSub;
		ros::Subscriber uav_pos_sub;
		ros::Subscriber battery_sub;
		ros::Subscriber clock_sub;
		rosgraph_msgs::Clock::ConstPtr time_msg;
		ros::Subscriber vbrac_surf_sub;
		ros::Subscriber vbrac_surf_inner_sub;
		ros::Subscriber followers_msgs_sub;
		ros::Subscriber global_poses_sub;
		ros::Subscriber points_to_leader_sub;
		ros::Subscriber goal_sub;
		ros::Subscriber alg_refresh_sub;
		ros::Subscriber test_cv_sub;

		
		//enoga::enoga command_for_leader;
		geometry_msgs::Point cur_pose; //текущая позиция (относительно точки старта БЛА, точки взлета)
		geometry_msgs::Point goal_pose; //заданная позиция (глобальная, относительно карты Gazebo)
		geometry_msgs::Point home_pose; //домашняя позиция
		geometry_msgs::Point global_pose; //глобальная позиция БЛА (относительно карты Gazebo)
		geometry_msgs::Point first_global_pose;
		geometry_msgs::Point leader_global_pose;
		std::vector<geometry_msgs::Point> waypoints; //точки маршрута
		size_t status=0; //0-nothing, 1-formation_alg, 2-vbrac_alg, 3-vbrac-surf текущий статус работы БЛА, 5 - летит на базу (например, после сигнала о низком заряде батареи)
		size_t cur_point=0;
		bool isSendNextCommandToFollower, isConnected=false;
		uint64_t startTime=0;
		double angle=0;
		double cur_vx=0;
		double cur_vy=0;
		geometry_msgs::Point dp;
		double leader_x=0, leader_y=0, leader_z=0;
		double rx=0, ry=0, rz=0;
		double turn_angle=0.;
		double x_turn=0, y_turn=0;
		double x_old_turn=0, y_old_turn=0;
		double previous_goal_x=0, previous_goal_y=0; 
		double critical_battery=0;
		int nOfUavs=0;

		// struct radVec {
		// 	double x;
		// 	double y;
		// 	double z;
		// 	double len;
		// 	double ang;
		// };
    	// std::vector<radVec> folRvectors;
		int leader_id=0;
		int first_uav_id=0;
		//double cur_vz;
		int id=0;
		double tolerance=0.1; //допустимое отклонение от заданной точки
		formation form;

		enoga::enoga enoga_command;
		bool isLeader=false;
		bool isFirst=false;
		std_msgs::Int8 command;
		int algorithm=0;
		double algParam=0;
		double algMinH=0;
		double algMaxH=0;
		bool caged=false;
		std::vector<cv::Point> longest_contour;
		std::vector<std::vector<double>> intrinsic_matrix={{476.56, 0, 399.595}, {0, 476.3, 399.09}, {0, 0, 1}};
	public:
	
		UAV(int ident);
		~UAV()=default;
		void handle_form(const enoga::enoga &enoga_value);
		void handle_surf_points(const enoga::enoga &enoga_value);
		
		//функция обработки сообщений от последователей
		void commands_from_followers_callback(const enoga::follower &follower_msg);
		void stateCallback(const mavros_msgs::State::ConstPtr &msg);
		void turn_clockwise();
		void turn_contrclockwise();
		void up();
		void down();
		void left();
		void right();
		void forward();
		void backward();
		void Offboard();
		void AutoLand();
		void arm();
		void disarm();
		void handle_commander(const std_msgs::Int8 &command_value);
		void init();
		void rosCycle();
		void leadposeCallback(const geometry_msgs::PoseStampedPtr& msg);
		void firstCallback(const geometry_msgs::PoseStampedPtr& msg);
		void swarmCallback(const enoga::follower& msg);
		void handle_follower_commander(const std_msgs::Int8 &command_value);
		void AUTO_RTL();
		void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
		double euclidian_distance(geometry_msgs::Point pose, geometry_msgs::Point goal_pose);
		void sameHeight();
		void handle_formation(const enoga::formationMsg/*std_msgs::Int8*/ &command);
		void formation_function();
		void poseCallback(const geometry_msgs::PoseStampedPtr &msg);
		void batteryCallback(const rosgraph_msgs::Clock::ConstPtr& time_msg);
		void lowBatteryLanding(const enoga::battery &b_msg);
		void vbrac_surf_callback(const enoga::points &points);
		void followers_msgs_callback(const enoga::follower& msg);
		void global_poses_callback(const gazebo_msgs::ModelStates& msg);
		void pointsCallback(const enoga::points &points);
		void findAngle();
		void radiusV();
		void turn();
		void goalToFollowers(const geometry_msgs::Point &msg);
		void changeAlgorithm(const enoga::algMsg &msg);
		void vbrac_surf_test_function();
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void masForGui(const rosgraph_msgs::Clock::ConstPtr& msg);
		void guiMasCallback(const enoga::guiMas &msg);
		void makeGroups();
		std::vector<double> getShiftVbrac(const int u, const int y);
};
#endif
