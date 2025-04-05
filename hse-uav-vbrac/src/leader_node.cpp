#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Point>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
//#include <mavros_msgs/TwistStamped.h>
//#include <>
#include <cstdio>
#include <tf/transform_datatypes.h>
#include <std_msgs/builtin_int8.h>
#include <vector>
#include <enoga/enoga.h>
#include <enoga/follower.h>
#include <map>

geometry_msgs::PoseStamped ps;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::State cur_state;
mavros_msgs::CommandBool arm_cmd;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Publisher local_pos_pub;
std_msgs::Int8  command_to_follower;
//enoga::enoga command_for_leader;
bool isSendNextCommandToFollower, isConnected=false;
bool isPosChanged=false;
double x=0;
double y=0;
double z=0;
double angle=0;
double cur_vx;
double cur_vy;
double cur_vz;
int id=0;
std::map<size_t, int> uavs;
enoga::follower last_follower_msg;

//TODO: пусть лидер хранит и отсылает позицию (высоту) первого БЛА формации, пусть также выстраивает формацию. Пусть при получении последнего сигнала и сам сменяет свою высоту
//TODO: при новой формации пусть значения uavs обнуляются, кроме лидера и первого БЛА построения
//TODO: сделать подсчет количества БЛА (поиск с регулярками по топикам?) для uavs
enoga::enoga enoga_command;
void handle_form(const enoga::enoga &enoga_value) {
	enoga_command=enoga_value;
	if(enoga_command.id==id) {
		ps.pose.position.x+=enoga_command.location.position.x;
		ps.pose.position.y+=enoga_command.location.position.y;
		ps.pose.position.z+=enoga_command.location.position.z;
		isSendNextCommandToFollower=true;
	}
}
/*void velocityCallback(const geometry_msgs::TwistStamped::ContsPtr& msg) {
	cur_vx=msg->twist.linear.x;
	cur_vy=msg->twist.linear.y;
	cur_vz=msg->twist.linear.z;
}*/

//функция обработки сообщений от последователей
void commands_from_followers_callback(const enoga::follower &follower_msg) {
	size_t flag=0;
	last_follower_msg=follower_msg;
	//TODO: если это был последний, то отправляет всем команду установки общей высоты
	//TODO: в дальнейшем предлагаю сделать отдельный топик или реорганизовать имеющийся, чтобы отсылалась позиция первого БЛА формации
	uavs[last_follower_msg.id]=1;
	for(const auto& [id, value] : uavs) {
		if(value==0)
			flag=1;
	}
	if(flag!=1) {
		command_to_follower.data=-9;
		isSendNextCommandToFollower=true;
	}
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
	cur_state=*msg;
}

//ps.pose.position.x=ps.pose.position.x+1*cos(angle);
//ps.pose.position.y=ps.pose.position.y+1*sin(angle);
void turn_clockwise() {
	angle-=1.571;
	ps.pose.orientation=tf::createQuaternionMsgFromYaw(angle);
	isPosChanged=true;
}

void turn_contrclockwise() {
	angle+=1.571;
	ps.pose.orientation=tf::createQuaternionMsgFromYaw(angle);
	isPosChanged=true;
}

void up() {
	ps.pose.position.z+=1;
	isPosChanged=true;
}

void down() {
	ps.pose.position.z-=1;
	isPosChanged=true;
}

//TODO: left и right не работают после 1 поворота против/по часовой стрелки(-е), проблема в синусе или косинусе?
void left() { //TODO: проверить, что адекватно работает при повороте, добавить смещение по x (так же поменять тр. функцию)
	ps.pose.position.y+=1*cos(angle);
	isPosChanged=true;
}

void right() {
	ps.pose.position.y-=1*cos(angle);
	isPosChanged=true;
}

void forward() {
	ps.pose.position.x+=1*cos(angle);
	ps.pose.position.y+=1*sin(angle);
	isPosChanged=true;
}

void backward() {
	ps.pose.position.x-=1*cos(angle);
	ps.pose.position.y-=1*sin(angle);
	isPosChanged=true;
}

void Offboard() {
	offb_set_mode.request.custom_mode="OFFBOARD";
	set_mode_client.call(offb_set_mode);
}

void AutoLand() {
	offb_set_mode.request.custom_mode="AUTO.LAND";
	set_mode_client.call(offb_set_mode);
}

void arm() {
	arm_cmd.request.value=true;
	arming_client.call(arm_cmd);
	if(arm_cmd.response.success)
		ROS_WARN_STREAM("Vehicle armed!");
}

void disarm() {
	arm_cmd.request.value=false;
	arming_client.call(arm_cmd);
	if(arm_cmd.response.success)
		ROS_WARN_STREAM("Vehicle disarmed!");
}

/*std::vector<std::vector<double>> formationMaker(size_t type, size_t amount, size_t parameter, double angle) {
	std::vector<std::vector<double>> allPos;
	double x=0.0;
	double y=0.0;
	allPos.push_back({x, y});
	for(size_t i=0; i<amount; ++i) {
		std::vector<double> pos={0.0, 0.0};
		switch(type) {
		case 0: { //klin
			if(i%2==0)
				x-=parameter;
			else
				x+=parameter;
			y-=parameter;
			pos[0]=x;
			pos[1]=y;
			allPos.push_back(pos);
		}
		default: {
		}
		}
	}
	return allPos;
}*/

//Делаем свое сообщение: координаты, айди дрона. Отправляем на commander
/*void formation(std::vector<std::vector<double>> cords) {
	int random=0;
	ros::NodeHandle ndh1;
	//template_gui_package::noga msg;
	geometry_msgs::PoseStamped localPos;
	//TODO smth for random first(skip it), just take cords
	for(size_t i=0; i<3; ++i) {
		//msg.flag=i;
		localPos.pose.position.x=cords[i][0];
		localPos.pose.position.y=cords[i][1];
		localPos.pose.position.z=i+1;
		//msg.location=localPos.pose;
		//noga_pub.publish(msg);
		std::string topic="/uav"+std::to_string(i)+"/mavros/setpoint_position/local";
		ros::Publisher localPosPub1=ndh1.advertise<geometry_msgs::PoseStamped>(topic, 10);
		localPosPub1.publish(localPos);
		ros::spinOnce();
	}
}*/

std_msgs::Int8 command;
void handle_commander(const std_msgs::Int8 &command_value) {
	command=command_value;
	switch(command.data) {
	case 1: {
		ROS_INFO("Vehicle armed");
		arm();
		break;
	}
	case 2: {
		ROS_INFO("Offboard");
		Offboard();
		break;
	}
	case 3: {
		up();
		if(isConnected) {
			command_to_follower.data=3; //up command
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 4: {
		down();
		if(isConnected) {
			command_to_follower.data=4; //down
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 5: {
		left();
		if(isConnected) {
			command_to_follower.data=5;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 6: {
		right();
		if(isConnected) {
			command_to_follower.data=6;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 7: {
		forward();
		//formation(formationMaker(0, 3, 5, 0));
		if(isConnected) {
			command_to_follower.data=7;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 8: {
		backward();
		if(isConnected) {
			command_to_follower.data=8;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 9: {
		turn_clockwise();
		if(isConnected) {
			command_to_follower.data=9;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case 10: {
		turn_contrclockwise();
		if(isConnected) {
			command_to_follower.data=10;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case -2: {
		AutoLand();
		if(isConnected) {
			command_to_follower.data=-2;
			isSendNextCommandToFollower=true;
		}
		break;
	}
	case -5: {
		isConnected=true;
		command_to_follower.data=-5;
		isSendNextCommandToFollower=true;
		break;
	}
	case -8: {
		//formtion
	}
	default: {
	}
	}
}

//TODO: добавить код для получения id нового лидера и сделать смену subscriber и publisher для нового лидера
//TODO: сделать подсчет кол-ва дронов на сцене, например, через подсчет топиков, характерных для одного дрона
int main(int argc, char **argv) {
	int dronesAmount=0;
	ros::init(argc, argv, "leader");
	ros::NodeHandle ndh;
	ros::Subscriber stateSub=ndh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, stateCallback);
	//ros::Subscriber followersSub=ndh.subscribe<enoga::enoga>("/enoga/for_leader_commands", 10, forLeaderCallback);
	ros::Publisher localPosPub=ndh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
	//ros::Publisher msgsForFollowers=ndh.advertise<enoga::enoga>("/enoga/follower_commands", 10);
	ros::Publisher commands_to_follower=ndh.advertise<std_msgs::Int8>("/commands_to_follower", 10);
	ros::Subscriber commandSub=ndh.subscribe("/commands_controller", 1000, handle_commander);
	arming_client=ndh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
	set_mode_client=ndh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
	ros::Subscriber enoga_sub=ndh.subscribe("/enoga_controller", 10, handle_form);
	ros::Subscriber followers_commands_sub=ndh.subscribe("/commands_to_leader", 10, commands_from_followers_callback);
	
	uavs[0]=1;
	uavs[1]=0;
	uavs[2]=0;

	isConnected=false;
	//geometry_msgs::PoseStamped pose;
	ps.pose.position.x=0;
	ps.pose.position.y=0;
	ps.pose.position.z=2;

	localPosPub.publish(ps);

	while(ros::ok()) {
		if(isSendNextCommandToFollower==true) {
			ROS_INFO("FollowerCommand");
			commands_to_follower.publish(command_to_follower);
			isSendNextCommandToFollower=false;
		}

		localPosPub.publish(ps);
		ros::spinOnce();
	}
}
