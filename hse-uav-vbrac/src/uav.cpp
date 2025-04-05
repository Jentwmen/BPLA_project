#include <uav.h>
//size_t uavAmount=0;
/* Готово: единая нода для последователей и лидера.
   Проверить: переключение лидера и первый БЛА построения


*/

//TODO: что делать, если лидер летит туда, где последователь? Лучше тогда вести последователя к точке по смещению, а не вслед за лидером
//TODO: очистить waypoints при полном полете
//TODO: почему-то летает по циклу в waypoints и при этом воспринимает другие команды
//Обработчик для получения точек маршрута лидером
void UAV::pointsCallback(const enoga::points &points) {
	for(size_t i=0; i<points.points.size(); ++i) {
		waypoints.push_back(points.points[i]);
	}
	status=4;
	if(isConnected) {
		command_to_follower.data=-8;
		isSendNextCommandToFollower=true;
	}
	ps.pose.position.x=waypoints[0].x;
	ps.pose.position.y=waypoints[0].y;
	ps.pose.position.z=waypoints[0].z;
	goal_pose.x=waypoints[0].x;
	goal_pose.y=waypoints[0].y;
	goal_pose.z=waypoints[0].z;  
	goal_to_follower_pub.publish(goal_pose);  
	if (!waypoints.empty()) {
        waypoints.erase(waypoints.begin());
    }
}

//Обработка сообщений с глобальными координатами БЛА (от Gazebo, лучше переделать в будущем под GPS, ГЛОНАСС и т.п.)
void UAV::global_poses_callback(const gazebo_msgs::ModelStates& msg) {
	std::string name="px4_lirs_"+std::to_string(id);
	std::string name1="px4_lirs_"+std::to_string(first_uav_id);
	std::string name2="px4_lirs_"+std::to_string(leader_id);
	int id_in_poses=-1;
	int first_id_in_poses=-1;
	int leader_id_in_poses=-1;
	for(size_t i=0; i<msg.name.size(); ++i) {
		if(msg.name[i]==name) {
			id_in_poses=i;
		} else if(msg.name[i]==name1) {
			first_id_in_poses=i;
		} else if(msg.name[i]==name2) {
			leader_id_in_poses=i;
		}
	}
	
	if(id_in_poses!=-1) {
		global_pose=msg.pose[id_in_poses].position;
	}
	
	if(isLeader) {
		first_global_pose=global_pose;
		leader_global_pose=global_pose;
	} else if(first_id_in_poses!=-1) {
		first_global_pose=msg.pose[first_id_in_poses].position;
	} else if(leader_id_in_poses!=-1) {
		leader_global_pose=msg.pose[leader_id_in_poses].position;
	}
}

//Подсчет дистанции между текущей точкой и трубуемой точкой. Необходимо для проверки того, достиг ли назначенной позиции БЛА в алгоритме формаций
double UAV::euclidian_distance(geometry_msgs::Point pose, geometry_msgs::Point goal_pose) {
	return sqrt(pow(goal_pose.x-pose.x, 2)+pow(goal_pose.y-pose.y, 2));
}

//Обработчик сообщений последователей (на лидере)
//TODO: разные случаи при ручном заходе на посадку и при севшей батарее, т.к., по идее, надо будет эту информацию сообщать оператору (для GUI, раздел Swarm Info)
void UAV::followers_msgs_callback(const enoga::follower& msg) {
	if(msg.command==1) { //взлетел БЛА-последователь, необходимо добавить в список активных БЛА
		auto it=std::find(active_uavs.begin(), active_uavs.end(), msg.id);
		if(it==active_uavs.end()) {
			active_uavs.push_back(msg.id);
			ROS_INFO("Takeoff: %d", msg.id);
		}
		uavs[msg.id]=0;
	} else if(msg.command==2) { //БЛА-последователь ушел на посадку, необходимо удалить БЛА из списка активных
		auto it = std::remove(active_uavs.begin(), active_uavs.end(), msg.id);
		active_uavs.erase(it, active_uavs.end());
		uavs.erase(msg.id);
		ROS_INFO("Landing: %d", msg.id);
	} else if(msg.command==3) { //БЛА-последователь ушел на посадку из-за низкого заряда батареи, необходимо удалить из списка активных
		auto it = std::remove(active_uavs.begin(), active_uavs.end(), msg.id);
		active_uavs.erase(it, active_uavs.end());
		uavs.erase(msg.id);
	}
}

//Устанавливает высоту лидера. После того, как все БЛА достигли нужной позиции. Работает для всех.
void UAV::sameHeight() {
	ps.pose.position.z=form.getZFirst();
}

//Конструктор, создающий объект класса с учетом идентификатора БЛА
UAV::UAV(int ident) {
	id=ident;
	if(ident==0) {
		isLeader=true;
		isFirst=true;
		commands_to_follower=ndh.advertise<std_msgs::Int8>("/commands_to_follower", 10);
		commandSub=ndh.subscribe("/commands_controller", 1000, &UAV::handle_commander, this);
		followers_commands_sub=ndh.subscribe("/commands_to_leader", 10, &UAV::commands_from_followers_callback, this);
		formSub=ndh.subscribe("/formation", 10, &UAV::handle_formation, this);
		enoga_pub=ndh.advertise<enoga::enoga>("/enoga_controller", 10);
		vbrac_surf_sub=ndh.subscribe("/vbrac_surf_points", 10, &UAV::vbrac_surf_callback, this);
		vbrac_surf_inner_pub=ndh.advertise<enoga::enoga>("/vbrac_surf_inner", 10);
		vbrac_bounding_points=ndh.advertise<enoga::points>("vbrac_surf_input", 10);
		vbrac_surf_inner_sub=ndh.subscribe("/vbrac_surf_inner", 10, &UAV::handle_surf_points, this);
		followers_msgs_sub=ndh.subscribe("/followers_msgs", 10, &UAV::followers_msgs_callback, this);
		points_to_leader_sub=ndh.subscribe("/coordinates_for_leader", 10, &UAV::pointsCallback, this);
		goal_to_follower_pub=ndh.advertise<geometry_msgs::Point>("/goal_to_followers",10);
		alg_refresh_sub=ndh.subscribe("/algorithm_params", 10, &UAV::changeAlgorithm, this);
		active_uavs.push_back(id);
		uavs[0]=1;
		
		test_cv_sub=ndh.subscribe("/px4_lirs_0/camera_/image_raw", 1000, &UAV::imageCallback, this);
	} else {
		goal_sub=ndh.subscribe("/goal_to_followers", 10, &UAV::goalToFollowers, this);
		commands_to_leader=ndh.advertise<enoga::follower>("commands_to_leader", 10);
		commands_to_follower_sub=ndh.subscribe("/commands_to_follower", 10, &UAV::handle_follower_commander, this);
		vbrac_surf_inner_sub=ndh.subscribe("/vbrac_surf_inner", 10, &UAV::handle_surf_points, this);
		followers_msgs_pub=ndh.advertise<enoga::follower>("/followers_msgs", 10);
	}
}


//обработчик полученных точек для внутренних БЛА согласно алгоритму VBRAC-SURF
//TODO: нужно разбить БЛА на пограничные и внутренние (часть Гриши), предлагаю использовать два вектора, где соответственно будут внешние и внутренние БЛА + вектор запасных (там храним только id)
void UAV::vbrac_surf_callback(const enoga::points &inner_points) {
	//points=inner_points.points;
	if(surf_points.size()==0||surf_points[0]!=inner_points.points[0]) { //если пришли значения, отличные от предыдущих
		surf_points=inner_points.points;
		enoga::enoga msg;
		for(size_t i=0; i<surf_points.size(); ++i) {
			msg.id=i;
			msg.location.position.x=surf_points[i].x;
			msg.location.position.y=surf_points[i].y;
			msg.location.position.z=i;
			vbrac_surf_inner_pub.publish(msg);
		}
	} else {
		//пришли такие же значения, что и раньше, ничего не делаем
	}
}

//Обработчик полученной точки внутреннего БЛА (только для внутренних)
//TODO: требует перевода на глобальные координаты
void UAV::handle_surf_points(const enoga::enoga &enoga_value) {
	enoga_command=enoga_value;
	/*if(enoga_command.id==id) {
		ps.pose.position.x=enoga_command.location.position.x;
		ps.pose.position.y=enoga_command.location.position.y;
		ps.pose.position.z=enoga_command.location.position.z;
		goal_pose.x=enoga_command.location.position.x;
		goal_pose.y=enoga_command.location.position.y;
		goal_pose.z=enoga_command.location.position.z;
		status=3;
	}*/
	if(enoga_command.id==id) {
		goal_pose.x=enoga_command.location.position.x;
		goal_pose.y=enoga_command.location.position.y;
		goal_pose.z=ps.pose.position.z+enoga_command.location.position.z;
		if(euclidian_distance(global_pose, goal_pose)>tolerance) {
			goal_pose.x=ps.pose.position.x+(goal_pose.x-global_pose.x);
		} else if(global_pose.x<goal_pose.x) {
			goal_pose.x=ps.pose.position.x+(goal_pose.x-global_pose.x);
		}
		
		if(global_pose.y>goal_pose.y) {
			goal_pose.y=ps.pose.position.y+(goal_pose.y-global_pose.y);
		} else if(global_pose.y<goal_pose.y) {
			goal_pose.y=ps.pose.position.y+(goal_pose.y-global_pose.y);
		}
		status=3;
		ps.pose.position.x=goal_pose.x;
		ps.pose.position.y=goal_pose.y;
		ps.pose.position.z=goal_pose.z;
	}
}


//обработчик события смены позиции БЛА. (Необходим для отправки сигнала лидеру о том, что БЛА с определенным id достиг позицию в алгоритме формаций). Работает для лидера и последователей.
void UAV::poseCallback(const geometry_msgs::PoseStampedPtr &msg) {
	cur_pose=msg->pose.position; //TODO: как это работает?) Почему-то работает в формациях (или где-то есть моя логическая ошибка, исправляющая баг? Но вообще имеет нулевые значения, только у msg->pose.position есть хоть какая-то вменяемая позиция
	if(status==1&&euclidian_distance(cur_pose, goal_pose)<=tolerance&&cur_vx==0.0&&cur_vy==0.0) { //БЛА в области заданной точки (formation)
		enoga::follower msg_for_leader;
		msg_for_leader.id=id;
		msg_for_leader.command=1;
		if(isLeader) {
			uavs[leader_id]=1;
		} else {
			commands_to_leader.publish(msg_for_leader);
		}
		status=0;
		ROS_INFO("Cur_pose: %f %f", msg->pose.position.x, msg->pose.position.y);
	} else if(status==2&&euclidian_distance(cur_pose, goal_pose)<=tolerance&&cur_vx==0.0&&cur_vy==0.0) { //БЛА подлетел к центру масс (vbrac)
	ROS_INFO("CM reached!");
	status=6;
	} else if(status==7&&euclidian_distance(cur_pose, goal_pose)<=tolerance&&cur_vx==0.0&&cur_vy==0.0) { //БЛА подлетел к следующей точке на границе (vbrac, не работает так, как надо! Механизм проверки работает, но полет по границе - не то)
		ROS_INFO("VBRAC point reached!");
		status=6;
	} else if(status==3&&euclidian_distance(cur_pose, goal_pose)<=tolerance&&cur_vx==0.0&&cur_vy==0.0) { //БЛА в области заданной точки (vbrac-surf) 
		//TODO: Отправка сигнала о достижении точки?
		ps.pose.position.z=leader_z;
		status=0;
	} else if(status==1) { //!!!! По сути это костыль для работы формаций. Пока не выяснил причину того, почему не работает переключение высот в начале. TODO: можно добавить промежуточный этап: переключение высот, потом полет к своим точкам, потом выравнивание высоты
		ps.pose.position.z=leader_z+id+1;
	} else if(status==4&&euclidian_distance(cur_pose, goal_pose)<=tolerance&&cur_vx==0.0&&cur_vy==0.0&&isLeader==true) {
		if (!waypoints.empty()) {
			goal_pose.x=waypoints[0].x;
			goal_pose.y=waypoints[0].y;
			goal_pose.z=waypoints[0].z;
			goal_to_follower_pub.publish(goal_pose);
			waypoints.erase(waypoints.begin());
		} else { 
			status=0;
			if(isConnected) {
				command_to_follower.data=-8;
				isSendNextCommandToFollower=true;
			}
		}
	} 
	/*else if(status==5&&sqrt(pow(home_pose.x-msg->pose.position.x, 2)+pow(home_pose.y-msg->pose.position.y, 2))<=tolerance&&cur_vx==0.0&&cur_vy==0.0) { //БЛА прилетел на позицию взлета (только высота у него другая)
		ROS_INFO("ED1: %f", sqrt(pow(0-0.837, 2)+pow(0-4.538, 2)));
		ROS_INFO("ED: %f", euclidian_distance(home_pose, msg->pose.position));
		ROS_INFO("Cur_pose: %f %f", msg->pose.position.x, msg->pose.position.y);
		ROS_INFO("Home_pose: %f %f", &home_pose.x, &home_pose.y);
		status=0;
		AutoLand();
	}*/
}

//Отправка координат точек каждого БЛА
void UAV::formation_function() {
	//TODO: сделать автоматическое назначение первого БЛА построения и повторную установку его позиции в form, если потребуется
	enoga::enoga msg;
	std::vector<std::vector<double>> cords=form.getPoints();
	//ps.pose.position.x=cords[0][0];
	//ps.pose.position.y=cords[0][1];
	for(size_t i=0; i<cords.size(); ++i) {
		msg.id=i;
		msg.location.position.x=cords[i][0];
		msg.location.position.y=cords[i][1];
		ROS_INFO("%d %f %f", i, cords[i][0], cords[i][1]);
		msg.location.position.z=form.getZFirst()+i+1;
		enoga_pub.publish(msg);
	}
}

//TODO: протестировать!
//обработчик команд на смену лидера или первого БЛА построения, на отправку БЛА на посадку и на добавление новых БЛА.
void UAV::swarmCallback(const enoga::follower& msg) {
	command_for_swarm=msg;
	std::string topic="";
	if(command_for_swarm.command==0) {
		if(leader_id==id) { //was leader, but not now
			leader_id=command_for_swarm.id;
			isLeader=false;
			commandSub.shutdown();
			commands_to_follower.shutdown();
			followers_commands_sub.shutdown();
			formSub.shutdown();
			vbrac_surf_sub.shutdown();
			vbrac_surf_inner_pub.shutdown();
			commands_to_leader=ndh.advertise<enoga::follower>("commands_to_leader", 10);
			commands_to_follower_sub=ndh.subscribe("/commands_to_follower", 10, &UAV::handle_follower_commander, this);
			vbrac_surf_inner_sub=ndh.subscribe("/vbrac_surf_inner", 10, &UAV::handle_surf_points, this);
			followers_msgs_sub.shutdown();
			followers_msgs_pub=ndh.advertise<enoga::follower>("/followers_msgs", 10);
			//make usbscriber of cmd for fol cmd	
		} else if(command_for_swarm.id==id) { //wasn't leader, but now it is
			leader_id=id;
			isLeader=true;
			commands_to_leader.shutdown();
			commands_to_follower_sub.shutdown();
			vbrac_surf_inner_sub.shutdown();
			commands_to_follower=ndh.advertise<std_msgs::Int8>("/commands_to_follower", 10);
			commandSub=ndh.subscribe("/commands_controller", 1000, &UAV::handle_commander, this);
			followers_commands_sub=ndh.subscribe("/commands_to_leader", 10, &UAV::commands_from_followers_callback, this);
			formSub=ndh.subscribe("/formation", 1000, &UAV::handle_formation, this);
			vbrac_surf_sub=ndh.subscribe("/vbrac_surf_points", 10, &UAV::vbrac_surf_callback, this);
			vbrac_surf_inner_pub=ndh.advertise<enoga::enoga>("/vbrac_surf_inner", 10);
			followers_msgs_pub.shutdown();
			followers_msgs_sub=ndh.subscribe("/followers_msgs", 10, &UAV::followers_msgs_callback, this);
		} else { //wasn't leader and now not a leader
			leader_id=command_for_swarm.id;
		}
		topic="/uav"+std::to_string(leader_id)+"/mavros/local_position/pose";
		leaderPosSub.shutdown();
		leaderPosSub=ndh.subscribe(topic, 10, &UAV::leadposeCallback, this);
	} else if(command_for_swarm.command==1) {
		if(first_uav_id==id) { //was first, but not now
			first_uav_id=command_for_swarm.id;
			isFirst=false;
			enoga_pub.shutdown();
		} else if(command_for_swarm.id==id) { //wasn't first, but now it is
			first_uav_id=id;
			isFirst=true;
			enoga_pub=ndh.advertise<enoga::enoga>("/enoga_controller", 10);
		} else { //wasn't first and now not first
			first_uav_id=command_for_swarm.id;
		}
		topic="/uav"+std::to_string(first_uav_id)+"/mavros/local_position/pose";
		firstPosSub.shutdown();
		firstPosSub=ndh.subscribe(topic, 10, &UAV::firstCallback, this);
	} else if(command_for_swarm.command==2) {
		//TODO: delete uav from uavs dict
	} else if(command_for_swarm.command==3) {
		//TODO: add uav in uavs dict
	}
}

//Обработчик команд алгоритма формаций. Формирует координаты точек построения и сохраняет их в объекте класса formation - form (поле класса UAV).
void UAV::handle_formation(const enoga::formationMsg &command) {
//	form.setNumber(3);
	form.setNumber(active_uavs.size());
//	ROS_INFO("Number was setted: %d", active_uavs.size());
	form.setParameter(command.parameter);
	form.setSectorAngle(command.secangle);
	form.setArcLength(command.arclength);
	if(form.getRotationAngle()!=command.formrotangle)
		form.setFormRotAngle(command.formrotangle);
	switch (command.form) {
		case 0:
		    form.generate_points_klin();
		    break;
		case 1:
    		form.generate_points_klin_with_first();
    		break;
		case 2:
		    form.generate_points_snake();
    		break;
		case 3:
		    form.generate_points_front();
		    break;
		case 4:
			form.generate_points_column();
    		break;
    	case 5:
    		form.generate_points_peleng();
			break;
		case 6:
			form.generate_points_flangs();
    		break;
		case 7:
		    form.generate_points_chess();
		    break;
		case 8:
    		form.generate_points_rect_full();
    		break;
		case 9:
		    form.generate_points_rect_empty();
		    break;
		case 10:
		    form.generate_points_romb();
		    break;
		case 11:
			form.generate_points_romb_empty();
    		break;
    	case 12:
    		form.generate_points_sector();
    		break;
		case 13:
		    form.generate_points_circle();
    		break;
  		default:{
  		}
  	}
  	ROS_INFO("Switch works fine!");
  	//form.changeRotationAngle(-command.formrotangle);
  	formation_function();
}

//Поулчение текущей скорости БЛА
void UAV::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	cur_vx=msg->twist.linear.x;
	cur_vy=msg->twist.linear.y;
}

//Получение позиции первого БЛА построения. Сохраняет в объект класса formation - form (поле класса UAV).
//Remove???
void UAV::firstCallback(const geometry_msgs::PoseStampedPtr &msg) {
  //first_x = msg->pose.position.x;
  //first_y = msg->pose.position.y;
  //first_z = msg->pose.position.z;
  //form.setXYZFirst(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  form.setXYZFirst(first_global_pose.x, first_global_pose.y, first_global_pose.z);
}
//////////////////
///////////////////

void UAV::goalToFollowers(const geometry_msgs::Point &msg) {
	previous_goal_x=goal_pose.x;
	previous_goal_y=goal_pose.y;
	goal_pose.x=msg.x;
	goal_pose.y=msg.y;

}

//Получение угла изменения (0 град лежат на оси X)
void UAV::findAngle() {
	double dx, dy;
	//double f, a;
	dx = goal_pose.x-previous_goal_x;
	dy = goal_pose.y-previous_goal_y;
	//f=dy/dx;
	turn_angle = atan2(dy, dx);
	ROS_INFO("dx, dy: %f, %f", dx, dy);
	ROS_INFO("angle: %f", turn_angle);
	// if ((dx > 0)&&(dy < 0)) {
	// 	a = atan2(dy,dx);
	// 	turn_angle=2*M_PI-a;
	// }
	// if ((dx < 0)&&(dy > 0)) {
	// 	a = atan2(dy,dx);
	// 	turn_angle=M_PI-a;
	// }
	// if ((dx < 0)&&(dy < 0)) {
	// 	a = atan2(dy,dx);
	// 	turn_angle=M_PI+a;
	// }
	// if ((dx == 0)&&(dy > 0)) {
	// 	turn_angle = 0.5*M_PI;
	// }
	// if ((dx == 0)&&(dy < 0)) {
	// 	turn_angle=1.5*M_PI;
	// }
	// if ((dx > 0)&&(dy == 0)) {
	// 	turn_angle=0.;
	// }
	// if ((dx < 0)&&(dy == 0)) {
	// 	turn_angle=M_PI;
	// }
}


//Рассчет изменения координат через полярную систему координат, где лидер принят за центр
void UAV::turn() {
	findAngle();
	double r;
	r = sqrt(rx*rx +ry*ry);
	x_turn = rx*cos(turn_angle)-ry*sin(turn_angle);
	y_turn = rx*sin(turn_angle)+ry*cos(turn_angle);
}


//Получение позиции лидера
void UAV::leadposeCallback(const geometry_msgs::PoseStampedPtr &msg) {
  	leader_x = msg->pose.position.x;
  	leader_y = msg->pose.position.y;
  	leader_z = msg->pose.position.z;

  if(status==4&&isLeader==false) {
	x_old_turn=x_turn;
	y_old_turn=y_turn; 
	turn();
  	ps.pose.position.x=x_turn-rx+leader_x;
  	ps.pose.position.y=y_turn-ry+leader_y;
	ps.pose.position.z=leader_z+id;
  }
 
    if(status==4&&isLeader==true) {
	ps.pose.position.x=goal_pose.x;
  	ps.pose.position.y=goal_pose.y;
	ps.pose.position.z=goal_pose.z;
  }
}

//Обработчик команд алгоритма формаций. Устанавливает БЛА новую позицию(ps.pose.position...), сохраняет назначенную позицию (goal_pose) и выставляет статус о том, что БЛА находится в полете до назначенной точки (status)
void UAV::handle_form(const enoga::enoga &enoga_value) {
	enoga_command=enoga_value;
	if(enoga_command.id==id) {
		//ROS_INFO("Cur pose in formation for next pos: %f, %f", cur_pose.x, cur_pose.y);
		//ROS_INFO("Goal pose with shift in formation for next pos: %f, %f", ps.pose.position.x, ps.pose.position.y);
		//ROS_INFO("Leader position: %f, %f", leader_x, leader_y);
		goal_pose.x=enoga_command.location.position.x;
		goal_pose.y=enoga_command.location.position.y;
		//goal_pose.z=enoga_command.location.position.z;
		if(!isFirst)
			goal_pose.z=enoga_command.location.position.z;
			//ps.pose.position.z=enoga_command.location.position.z;
		else
			goal_pose.z=ps.pose.position.z;
			//ps.pose.position.z=ps.pose.position.z;

		ROS_INFO("GoalPoseGlobal: %f %f %f", goal_pose.x, goal_pose.y, goal_pose.z);
		
		if(euclidian_distance(global_pose, goal_pose)>tolerance) {
			if(global_pose.x>goal_pose.x) {
				goal_pose.x=ps.pose.position.x+(goal_pose.x-global_pose.x);
			} else if(global_pose.x<goal_pose.x) {
				goal_pose.x=ps.pose.position.x+(goal_pose.x-global_pose.x);
			}
			
			if(global_pose.y>goal_pose.y) {
				goal_pose.y=ps.pose.position.y+(goal_pose.y-global_pose.y);
			} else if(global_pose.y<goal_pose.y) {
				goal_pose.y=ps.pose.position.y+(goal_pose.y-global_pose.y);
			}
			ROS_INFO("GoalPoseLocal: %f %f %f", goal_pose.x, goal_pose.y, goal_pose.z);
			ps.pose.position.x=goal_pose.x;
			ps.pose.position.y=goal_pose.y;
			ps.pose.position.z=goal_pose.z;
			//ROS_INFO("PSPosePosition: %f %f %f", ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
			
		}
		//ps.pose.position.x=enoga_command.location.position.x;
		//ps.pose.position.y=enoga_command.location.position.y;
		status=1;
		//isSendNextCommandToFollower=true;
	}
}

//Обработчик команд от последователей к первому БЛА построения. Обновляет информацию о БЛА построения (если value в цикле равен 0, то позиции не достиг, если 1, то позиция была достигнута) в uavs.
//Если все достигли свои позиции, то оставляет flag=0 и посылает всем членам роя команду выставления единой высоты - sameHeight()
void UAV::commands_from_followers_callback(const enoga::follower &follower_msg) {
	size_t flag=0;
	last_follower_msg=follower_msg;
	uavs[last_follower_msg.id]=1;
	for(const auto& [id, value] : uavs) {
		if(value==0)
			flag=1;
	}
	if(flag!=1) {
		ROS_INFO("Send a same height command");
		command_to_follower.data=-9;
		isSendNextCommandToFollower=true;
		//TODO: необходимо обнулить uavs
	}
}

//Обработчик изменения состояния БЛА (см. Offboard, AUTO.LAND и т.п.)
void UAV::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
	cur_state=*msg;
}


double batteryDischargeFunction(float initialVoltage, long time, float internalResistance, float capacity) {
	double t = time/1000000000;
    return initialVoltage * exp(-(t) / (internalResistance * capacity));
}

static int firstCall=0;

void UAV::batteryCallback(const rosgraph_msgs::Clock::ConstPtr& time_msg) { // const enoga::battery& battery_msg
	
	// Параметры для симуляции батареи
    float initialVoltage = 14.8-3.8; //Начальное напряжение - крайнее напряжение
    float internalResistance = 0.44; //Внутреннее сопротивление аккумулятора
    float capacity = 5000; //Емкость

	enoga::battery b_msg;
	
	battery_pub=ndh.advertise<enoga::battery>("/uav"+std::to_string(id)+"/battery", 10);
	
	//std::cout << isFirstCallback;
    if((cur_state.mode == "OFFBOARD")||(cur_state.mode == "AUTO.RTL")) {
		static const long startTime = (time_msg->clock.toNSec()) + (time_msg->clock.toSec())*1000000000;
		long currentTime = (time_msg->clock.toNSec()) + (time_msg->clock.toSec())*1000000000;
		long duration = currentTime - startTime;
		b_msg.start_time = startTime;
		b_msg.current_time = currentTime;
		b_msg.duration = duration;
    	double percentage = batteryDischargeFunction(initialVoltage, duration, internalResistance, capacity) * 100 / initialVoltage;
    	percentage=percentage;
		b_msg.percentage = percentage;
		battery_pub.publish(b_msg);
		per_save = percentage;
    }
}

static int fl=0; //Флаг для проверки первого вызова
static long st_time=0;
static double x = 0;
static double y = 0;
static double z = 0;
static double critical_battery=0;

void UAV::lowBatteryLanding(const enoga::battery &b_msg) {
	static const geometry_msgs::Point start_pose = cur_pose;
	if ((abs(cur_pose.x-start_pose.x) > 3)||(abs(cur_pose.y-start_pose.y) > 3)||(abs(cur_pose.z-start_pose.z) > 3)){
		if (st_time==0) {
			st_time = b_msg.duration;
			x = cur_pose.x;
			y = cur_pose.y;
			z = cur_pose.z;
		}
		long c_time = b_msg.duration;
		double t = (c_time-st_time)/1000000000;

		if (((abs(cur_pose.x-x) < 2)&&(abs(cur_pose.y-y) < 2)&&(abs(cur_pose.z-z) < 2))
		   &&(t > 15)&&(fl!=1)) {
			critical_battery=100-(b_msg.percentage);
		}
		else if (t > 15) {
			st_time = 0;
			t = 0;
		}
		if (critical_battery!=0) {
			const static double c_b = critical_battery;
			double c_bat=b_msg.percentage;
			if (c_bat<(c_b+15) && fl!=1) {
				fl++;
				AUTO_RTL();
				//std::cout << "("<<c_b<<")";
			}
		}
	}
}

//Функция поворота по часовой стрелке на 90 градусов
void UAV::turn_clockwise() {
	angle+=1.571;
	ps.pose.orientation=tf::createQuaternionMsgFromYaw(angle);
}

//Функция поворота против часовой стрелки на 90 градусов
void UAV::turn_contrclockwise() {
	angle-=1.571;
	ps.pose.orientation=tf::createQuaternionMsgFromYaw(angle);
	//ROS_INFO("Current position: x- %f, y- %f, z- %f", cur_pose.x, cur_pose.y, cur_pose.z);
	//ROS_INFO("Global position first: %f, %f, %f", first_global_pose.x, first_global_pose.y, first_global_pose.z);
}

void UAV::up() {
	ps.pose.position.z+=1;
}

void UAV::down() {
	ps.pose.position.z-=1;
}

void UAV::left() {
	ps.pose.position.y+=1*cos(angle);
	ps.pose.position.x-=1*sin(angle);
}

void UAV::right() {
	ps.pose.position.y-=1*cos(angle);
	ps.pose.position.x+=1*sin(angle);
}

void UAV::forward() {
	ROS_INFO("Forward_function");
	ps.pose.position.x+=1*cos(angle);
	ps.pose.position.y+=1*sin(angle);
	//ROS_INFO("%f, %f", ps.pose.position.x, ps.pose.position.y);
}

void UAV::backward() {
	ps.pose.position.x-=1*cos(angle);
	ps.pose.position.y-=1*sin(angle);
}

void UAV::Offboard() {
	//ps.pose.position.x=1;
	//ps.pose.position.y=id*2;
	ps.pose.position.z=2;
	offb_set_mode.request.custom_mode="OFFBOARD";
	set_mode_client.call(offb_set_mode);
	mavros_msgs::HomePosition home_msg;
	home_pose.x=cur_pose.x+id;
	home_pose.y=cur_pose.y;
	//home_pose.z=0;
	//home_msg.position.coordinate_frame=mavros_msgs::HomePosition::FRAME_GLOBAL_INT;
	//home_msg.position.x=ps.pose.position.x;
	//home_msg.position.y=ps.pose.position.y;
	//home_msg.position.z=ps.pose.position.z;
	home_pub.publish(home_msg);
	ROS_INFO("HP_X: %f", &home_pose.x);
	ROS_INFO("HP_Y: %f", &home_pose.y);
	if(!isLeader) {
		enoga::follower followerMsg;
    	followerMsg.id=id;
    	followerMsg.command=1;
    	followers_msgs_pub.publish(followerMsg);
    }
}

void UAV::AutoLand() {
	offb_set_mode.request.custom_mode="AUTO.LAND";
	set_mode_client.call(offb_set_mode);
	if(!isLeader) {
		enoga::follower followerMsg;
    	followerMsg.id=id;
    	followerMsg.command=2;
	    followers_msgs_pub.publish(followerMsg);
	}
}

void UAV::arm() {
	arm_cmd.request.value=true;
	arming_client.call(arm_cmd);
	if(arm_cmd.response.success)
		ROS_WARN_STREAM("Vehicle armed!");
}

void UAV::disarm() {
	arm_cmd.request.value=false;
	arming_client.call(arm_cmd);
	if(arm_cmd.response.success)
		ROS_WARN_STREAM("Vehicle disarmed!");
}

void UAV::radiusV() {
	 rx = global_pose.x - leader_global_pose.x;
	 ry = global_pose.y - leader_global_pose.y;
	 //rz = cur_pose.z - leader_z;
}

//TODO: разобраться, как поставить домашнюю точку, т.к. сейчас просто приземляется в текущей точке, а должен уходить на посадку в точке взлета
void UAV::AUTO_RTL() {
  offb_set_mode.request.custom_mode = "AUTO.RTL";
  set_mode_client.call(offb_set_mode);
  //status=5;
  //ps.pose.position.x=home_pose.x;
  //ps.pose.position.y=home_pose.y;
  //goal_pose.x=home_pose.x;
  //goal_pose.y=home_pose.y;
  //ps.pose.position.z=0;
  if (offb_set_mode.response.mode_sent)
    ROS_WARN_STREAM("AUTO.RTL enabled");
    //ROS_INFO("Baza");
  if(!isLeader) {
  	enoga::follower followerMsg;
    followerMsg.id=id;
    followerMsg.command=3;
    followers_msgs_pub.publish(followerMsg);
  }
}

//Обработка полученных параметров алгоритма от оператора
void UAV::changeAlgorithm(const enoga::algMsg &msg) {
	algorithm=msg.type;
	algParam=msg.param;
	algMinH=msg.minH;
	algMaxH=msg.maxH;
	critical_battery=msg.landBat;
	nOfUavs=msg.numb;
	ROS_INFO("%d %f %d %d %f", msg.type, msg.param, msg.minH, msg.maxH, msg.landBat);
	if(algorithm==1) {
	//Вернуть и переделать переключение на vbrac-surf
	//	vbrac_surf_test_function();
		caged=false;
	} else if(algorithm==0) {
		//
	}
}

//Обработчик команд, посылаемых лидером последователям
void UAV::handle_follower_commander(const std_msgs::Int8 &command_value) {
  command = command_value;
  switch (command.data) {
  case 1: {
    arm();
    break;
  }
  case 2: {
    Offboard();
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
  case 10: {
    turn_contrclockwise();
    break;
  }
  case -1: {
    AUTO_RTL();
    break;
  }
  case -2: {
    AutoLand();
    break;
  }
  case -5: {
    Offboard();
    arm();
	radiusV();
    // ros::Duration(3.0).sleep();
    // OFFBOARD();
    // ros::Duration(3.0).sleep();
    // follow();
    break;
  }
  //case -6: { Это нужно, чтобы при отключении последователей они ушли на посадку, сейчас они зависнут в воздухе (на своей позиции)
  //  AutoLand();
  //  break;
  //}
  case -7: {
    //follow();
    break;
  }
  case -8: {
  	//formation
  	if(status!=4)
  		status=4;
  	else if(status==4)
  		status=0;
  }
  case -9: {
  	sameHeight();
  }
  default: {
  }
  }
}

//Обработчик команд, посылаемых оператором лидеру (через GUI)
void UAV::handle_commander(const std_msgs::Int8 &command_value) {
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
		ROS_INFO("Forward");
		forward();
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
	case -1: {
		AUTO_RTL();
		if(isConnected) {
			command_to_follower.data=-1;
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
	case -6: {
		isConnected=false;
		//isSendNextCommandToFollower=true; //нужно, чтобы после отключения последователей они ушли на посадку
		//command_to_follower.data=-6;
		break;
	}
	case -8: {
		//formtion
	}
	case -9:
		sameHeight();
	default: {
	}
	}
}

void UAV::guiMasCallback(const enoga::guiMas &msg){
	datgui=msg;
}


void UAV::masForGui(const rosgraph_msgs::Clock::ConstPtr& msg) {
	datgui.batteries[id]=per_save;
	if(cur_state.armed){
		datgui.states[id]=1;
	}
	else {
		datgui.states[id]=0;
	}
	guimas_pub.publish(datgui);
	makeGroups();
	//std::cout << "(0)";
}

//Инициализация объекта класса UAV. Инициализация БЛА для работы с ним
void UAV::init() { //TODO: uavs[i] доделать
	std::string topic="/uav"+std::to_string(id)+"/mavros/state";
	stateSub=ndh.subscribe<mavros_msgs::State>(/*"/uav0/mavros/state"*/topic, 10, &UAV::stateCallback, this);
	//ros::Subscriber followersSub=ndh.subscribe<enoga::enoga>("/enoga/for_leader_commands", 10, forLeaderCallback);
	topic="/uav"+std::to_string(id)+"/mavros/setpoint_position/local";
	local_pos_pub=ndh.advertise<geometry_msgs::PoseStamped>(/*"/uav0/mavros/setpoint_position/local"*/topic, 10);
	//ros::Publisher msgsForFollowers=ndh.advertise<enoga::enoga>("/enoga/follower_commands", 10);
	topic="/uav"+std::to_string(id)+"/mavros/cmd/arming";
	arming_client=ndh.serviceClient<mavros_msgs::CommandBool>(/*"/uav0/mavros/cmd/arming"*/topic);
	topic="/uav"+std::to_string(id)+"/mavros/set_mode";
	set_mode_client=ndh.serviceClient<mavros_msgs::SetMode>(/*"/uav0/mavros/set_mode"*/topic);
	enoga_sub=ndh.subscribe("/enoga_controller", 10, &UAV::handle_form, this);
	swarmSub=ndh.subscribe("/swarm_controller", 10, &UAV::swarmCallback, this);
	topic="/uav"+std::to_string(leader_id)+"/mavros/local_position/pose";
	leaderPosSub=ndh.subscribe(topic, 10, &UAV::leadposeCallback, this);
	topic="/uav"+std::to_string(first_uav_id)+"/mavros/local_position/pose";
	firstPosSub=ndh.subscribe(topic, 10, &UAV::firstCallback, this);
	topic="/uav"+std::to_string(id)+"/mavros/local_position/pose";
	uav_pos_sub=ndh.subscribe(topic, 10, &UAV::poseCallback, this);
	topic="/clock";
	clock_sub=ndh.subscribe(topic, 10, &UAV::batteryCallback, this);
	guimas_sub=ndh.subscribe(topic, 10, &UAV::masForGui, this);
	topic="/gui_info";
	guiCallback_sub=ndh.subscribe(topic, 10, &UAV::guiMasCallback, this);
	topic="/uav"+std::to_string(id)+"/battery";
	//battery_pub=ndh.advertise<enoga::battery>(topic, 10); //Доработать работу со временем, провести тесты, написать функцию посадки при низкой батарее, обновить README
	battery_sub=ndh.subscribe(topic, 10, &UAV::lowBatteryLanding, this);
	topic="/uav"+std::to_string(id)+"/mavros/home_position/set";
	home_pub=ndh.advertise<mavros_msgs::HomePosition>(topic, 10);
	topic="/gazebo/model_states";
	global_poses_sub=ndh.subscribe(topic, 10, &UAV::global_poses_callback, this);
	guimas_pub=ndh.advertise<enoga::guiMas>("/gui_info", 10);
	for (int i=0; i<100; i++) {
        datgui.batteries.push_back(0.);
		datgui.states.push_back(0);
    }
	guimas_pub.publish(datgui);
	makeGroups();
	
	//int fl =0;
	startTime = 0;
	//uavs[0]=1;
	//uavs[1]=0;
	//uavs[2]=0;
	isConnected=false;
	
	//TODO: Убрать, это костыль на тест вбрака (opencv stage)
	//if(id==0) {
	//	ps.pose.position.x=62;
	//	ps.pose.position.y=39;
	//	ps.pose.position.z=3;
	//}
	//ps.pose.position.x=0;
	//ps.pose.position.y=id*2;
	//ps.pose.position.z=2;
	local_pos_pub.publish(ps);


	//battery_pub.publish(battery_msg); 
}

//Цикл ros, который надо запускать в while мейна, а заканчивать ros::spinOnce		
void UAV::rosCycle() {
	if(isSendNextCommandToFollower==true&&isLeader==true) {
		ROS_INFO("FollowerCommand");
		commands_to_follower.publish(command_to_follower);
		isSendNextCommandToFollower=false;
	}
	ps.pose.orientation=tf::createQuaternionMsgFromYaw(angle);
	//ROS_INFO("%f, %f", ps.pose.position.x, ps.pose.position.y);
	local_pos_pub.publish(ps);
}

//иммитация VBRAC-SURF
void UAV::vbrac_surf_test_function() {
	std::vector<geometry_msgs::Point> points;
	geometry_msgs::Point one;
	geometry_msgs::Point two;
	geometry_msgs::Point three;
	geometry_msgs::Point four;
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
	surf_points=points;
  	enoga::points msg;
	/*for(size_t i=0; i<surf_points.size(); ++i) {
		msg.id=i;
		msg.location.position.x=surf_points[i].x;
		msg.location.position.y=surf_points[i].y;
		msg.location.position.z=i;
		//vbrac_surf_inner_pub.publish(msg);
	}*/
	
	for(std::vector<geometry_msgs::Point>::iterator it=points.begin(); it!=points.end(); ++it) {
  		geometry_msgs::Point point;
  		point.x=(*it).x;
  		point.y=(*it).y;
  		point.z=0;
  		msg.points.push_back(point);
  	}
	vbrac_bounding_points.publish(msg);
}

//Получение смещения на центр масс границы зоны наводнения
std::vector<double> UAV::getShiftVbrac(const int u, const int v) {
	std::vector<double> point;
	double x=0;
	double y=0;
	
	x=global_pose.z*((u-intrinsic_matrix[0][2])/intrinsic_matrix[0][0]);
	y=global_pose.z*((v-intrinsic_matrix[1][2])/intrinsic_matrix[1][1]);
	point.push_back(x);
	point.push_back(y);
	return point;
}

//Функция обработки изображения с камеры БЛА (только с нижней)
void UAV::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat image=cv_bridge::toCvShare(msg, "bgr8")->image;
	
	// Переводим полученное изображение из RGB в HSV и формируем маску воды (синей области)
	cv::Mat hsv;
	cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
	cv::Scalar lower_blue(90, 50, 50); //90 50 50
	cv::Scalar upper_blue(140, 255, 255); // 140 255 255
	cv::Mat water_mask;
	cv::inRange(hsv, lower_blue, upper_blue, water_mask);
	cv::GaussianBlur(water_mask, water_mask, cv::Size(5,5), 0);
	
	// При помощи Canny получаем границы белых объектов (с маской выходит, что вода - белая, а все остальное - черное и игнорируется, как бэкграунд, не интересующий нас)
	cv::Mat edges;
	cv::Canny(water_mask, edges, 100, 200);
	
	// Получаем контуры объектов из изображения с границами белых объектов, далее (внизу) производим их отрисовку на копии от исходного изображения
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::Mat output =image.clone();
	if(algorithm==1) {
		//cv::drawContours(output, contours, -1, cv::Scalar(0,0,255), 2);
		
		if(contours.size()!=0) {
			double max_length=0;
			double img_center_x=image.cols/2;
			double img_center_y=image.rows/2;
			double angle_x=0;
			double angle_y=0;
			double hor_fov=0;
			double ver_fov=1.3962634;
			hor_fov=ver_fov;
			double distance_x=0;
			double distance_y=0;
			
			double length=0;
			for(size_t i=0; i<contours.size(); ++i) {
				length=cv::arcLength(contours[i], true);
				if(length>max_length) {
					max_length=length;
					longest_contour=contours[i];
				}
			}
			
			// Находим центр масс (который попадает на границу воды), отрисовываем его
			cv::Moments m=cv::moments(longest_contour);//contours[0]);
			int cx=int(m.m10/m.m00);
			int cy=int(m.m01/m.m00);
			cv::circle(output, cv::Point(cx,cy), 5, cv::Scalar(0,0,255), -1);
			cv::circle(output, cv::Point(longest_contour[0].x,longest_contour[0].y), 5, cv::Scalar(255,0,0), -1);
			//cv::circle(output, cv::Point(longest_contour[longest_contour.size()/2 +20].x,longest_contour[longest_contour.size()/2 +20].y), 5, cv::Scalar(0,255,0), -1);
			
			//чтобы не спамить точками, нашли - посчитали - полетели
			if(caged==false) {
				caged=true;
				std::vector<double> new_point=getShiftVbrac(cx, cy);
				//ps.pose.position.x=ps.pose.position.x-(ps.pose.position.z*((cx-intrinsic_matrix[0][2])/intrinsic_matrix[0][0]));
				//ps.pose.position.y=ps.pose.position.y-(ps.pose.position.z*((cx-intrinsic_matrix[1][2])/intrinsic_matrix[1][1]));
				ps.pose.position.x=ps.pose.position.x-(new_point[0]);//*cos(angle));
				ps.pose.position.y=ps.pose.position.y-(new_point[1]);//*sin(angle));
				goal_pose=ps.pose.position;
				status=2; // Для обработчика позиции БЛА, чтобы там понимать, что БЛА должен достигнуть центра масс
				ROS_INFO("local_z: %f ; global_z: %f", cur_pose.z, global_pose.z);
			}
			if(status==6) {
				//TODO: Не работает. Выполняется, когда БЛА достиг центра масс
				std::vector<double> new_point=getShiftVbrac(longest_contour[0].y, longest_contour[0].x);
				ps.pose.position.x=ps.pose.position.x-(new_point[0]);//*sin(angle));
				ps.pose.position.y=ps.pose.position.y-(new_point[1]);//*cos(angle));
				goal_pose=ps.pose.position;
				status=7; // Для обработчика позиции БЛА, чтобы там понимать, что БЛА летит к следующей точке на границе (работает ужасно - не работает!)
			}
		}
	}
	// Отрисовываем самый длинный контур - предположительно граница воды
	if(longest_contour.size()!=0) {
		cv::drawContours(output, std::vector<std::vector<cv::Point>>(1, longest_contour), -1, cv::Scalar(0,0,255), 2);
	}
	imshow("image", output);//output);
	cv::waitKey(30);
}

void UAV::makeGroups() {
    groups_pub=ndh.advertise<enoga::groups>("/groups", 10);
    int col=nOfUavs;
    int n_out, n_in, n_res = 0;
	int fl_1, fl_2 = 0;
	groupsmsg.outer.clear();
	groupsmsg.inner.clear();
	groupsmsg.reserve.clear();
    if(col >= 4){
      	for(int i=0; i < (col)*0.4; i++) {
        	if(i!=leader_id) {
          		groupsmsg.outer.push_back(i);
        	}
		}
		for(int i=(col)*0.4+1; i < (col)*0.8; i++) {
        	if(i!=leader_id) {
          		groupsmsg.inner.push_back(i);
        	}
		}
		for(int i=(col)*0.8+1; i < (col); i++) {
        	if(i!=leader_id) {
          		groupsmsg.reserve.push_back(i);
        	}
		}
    }
	if (col == 3){
		for(int i=0; i < col-1; i++) {
			if((i!=leader_id)&&(fl_1==0)) {
				groupsmsg.outer.push_back(i);
				fl_1=1;
			}
			if((i!=leader_id)&&(fl_2==0)) {
				groupsmsg.inner.push_back(i);
				fl_2=1;
			}
		}
	}
  	groups_pub.publish(groupsmsg);
}
