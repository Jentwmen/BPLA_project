#ifndef RQT_UAV_GUI_CPP_UAV_PLUGIN_H
#define RQT_UAV_GUI_CPP_UAV_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_uav_gui_cpp/ui_gui_plugin.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/macros.h>
#include <ros/master.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core/core.hpp>
#include <QImage>
#include <QList>
#include <QString>
#include <QStringList>
#include <QSet>
#include <QSize>
#include <QWidget>
#include <vector>
#include <qwt/qwt_slider.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_symbol.h>
#include <QGraphicsScene>
#include <QPointF>
#include <rqt_uav_gui_cpp/formation.h>
#include <QPen>
#include <QBrush>
#include <QComboBox>
#include <QtGlobal>
#include <QDoubleSpinBox>
#include <enoga/formationMsg.h>
#include <QGraphicsLineItem>
#include <QGraphicsTextItem>
#include <QToolTip>
#include <QDirIterator>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/builtin_int8.h>
#include <enoga/points.h>
#include <map>
#include <string>
#include <enoga/battery.h>
#include <enoga/guiMas.h>
#include <enoga/groups.h>
#include <mavros_msgs/HomePosition.h>
#include <gazebo_msgs/ModelStates.h>

#include "iostream"
#include <enoga/enoga.h>
#include <algorithm>
#include <vector>
#include <QTableWidget>
#include <QTimer>
#include <QTime>
#include <QSettings>

namespace rqt_uav_gui_cpp
{

class GUIPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  GUIPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);
  virtual void initFormations();
  virtual void controlSettingsSave();
  virtual void controlSettingsLoad();

protected slots:
  virtual void updateTopicList();
  virtual void onAddCheckpointPressed();
  virtual void onStartPressed();
  virtual void updateTable();
  virtual void updateProgressBar();

protected:
  virtual QSet<QString> getTopics(const QSet<QString>& message_types,/*const QSet<QString>& message_sub_types,*/ const QList<QString>& transports);
  virtual void selectTopic(const QString& topic);
//  virtual void selectTopic2(const QString& topic);
  
protected slots:
  virtual void onTopicChanged(int index);
  //virtual void onTopicChanged2(int index);
  //virtual void saveImage();
  //virtual void onLand();
  //virtual void onTakeoff();
  virtual void onUp();
  virtual void onDown();
  virtual void onYawL();
  virtual void onYawR();
  virtual void onFrw();
  virtual void onBack();
  virtual void onRollL();
  virtual void onRollR();
  virtual void onArm();
  virtual void onOffboardOn();
  virtual void onStopPressed();
  virtual void onHomePressed();
  virtual void autoland();
  virtual void setFormation();
  virtual void plusGradus();
  virtual void minusGradus();
  virtual void refreshForm(QString text);
  virtual void updateFormParameter();
  virtual void updateFormSecAngle();
  virtual void showFormationHelpMessage();
  virtual void batteryLevelChanged(int nValue);
  virtual void refreshAlgorithm();
  

protected:
  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void callbackForwImage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void gui_msgCallback(const enoga::guiMas& msg);
  virtual void groups_Callback(const enoga::groups& msg);
//  virtual void callbackImage2(const sensor_msgs::Image::ConstPtr& msg);
//  virtual void invertPixels(int x, int y);
//  virtual void invertPixels2(int x, int y);
  
private:
  Ui::MainWindow ui_;
  QSettings* settings;
  QList<QString> topicsfw;
  QWidget* widget_;
  QFrame *frame;
  QGraphicsView *formationGraphics;
  QGraphicsScene *formationScene;
  QTimer *timer;
  ros::Subscriber gui_sub;
  ros::Subscriber groups_sub;
  ros::Subscriber battery_sub;
  ros::Subscriber battery_own_sub;
  ros::Subscriber state_sub;
  ros::Subscriber destination_check;
  ros::Subscriber coordinates_sub;
  QString rosData;
  image_transport::Subscriber subscriber_, forw_subscriber_;
  //image_transport::Subscriber subscriber1_, subscriber2_;
  cv::Mat conversion_mat_, bin_mat_, forw_conversion_mat_;
//  cv::Mat conversion_mat1_, conversion_mat2_;
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Publisher formPub;
  ros::Publisher waypoints_pub;
  ros::Publisher algParam_pub;
  enoga::formationMsg formCmd;
  enoga::points points_msg;
  std::vector<int> outer_ids;
	std::vector<int> inner_ids;
	std::vector<int> reserve_ids;
  formation form;
  std::vector<float> bat_v;
	std::vector<float> stat_v;
  bool armPressed = false;
  int leader_id = 0; //<-- заменить на функцию получающую id leader'а
  double angle=0;
  int fl=0;
  int k = 0;
  int numberOfUavs = 0;
  struct algorithmDetails {
  	char type='0';
  	int minH=0;
  	int maxH=0;
  	double landBat=0;
  	double param=0;
  } algDet;
};

}  // namespace
#endif
