#include <uav.h>

//size_t uavAmount=0;
/* Готово: единая нода для последователей и лидера.
   Проверить: переключение лидера и первый БЛА построения
   Добавить: 2) Починить команды влево и вправо при повороте на 90 градусов вправо или влево,


*/
size_t getUavAmount() {
	size_t amount=0;
	ros::master::V_TopicInfo topic_infos;
	ros::master::getTopics(topic_infos);
	std::string prevVal="";
	for(ros::master::V_TopicInfo::iterator it=topic_infos.begin(); it!=topic_infos.end(); ++it) {
		const ros::master::TopicInfo& info=*it;
		if(info.name.find("uav"+std::to_string(amount))!=std::string::npos) {
				++amount;
		}
	}
	
	return amount;
	//uavAmount=amount;
}

//TODO: отправляем команду лидеру и получаем номер нового дрона, у которого нет управляющей ноды (с ним сформировать строку и передать 
int main(int argc, char **argv) {
	std::string nodeName="participant"+std::string(argv[1]);//std::to_string(uavAmount);
	ros::init(argc, argv, nodeName); //leader was
	//getUavAmount();
	//std::vector<UAV> uavs;
	//for(size_t i=0; i<uavAmount; ++i) {
		//UAV testUav(i);
		//testUav.init();
		//testUav.rosCycle();
		//uavs.push_back(testUav);
	//}
	UAV testUav(std::atoi(argv[1]));//uavAmount);
	testUav.init();
	//uavAmount+=1;
	ros::Rate rate(20.0);
	while(ros::ok()) {
		testUav.rosCycle();
		ros::spinOnce();
		rate.sleep();
	}
}
