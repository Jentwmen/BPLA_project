#include "rqt_uav_gui_cpp/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMessageBox>
#include <QPainter>
#include <QPalette>
#include <QColor>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include <enoga/algMsg.h>

namespace rqt_uav_gui_cpp
{

GUIPlugin::GUIPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("UAV GUI");
  timer = new QTimer();
  connect(timer, SIGNAL(timeout()), this, SLOT(updateTable()));
  timer->start(100);
}

void GUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  pub = nh.advertise<std_msgs::Int8>("/commands_controller", 10);
  //pub = nh.advertise<std_msgs::Char>("gui_topic", 10);
  formPub=nh.advertise<enoga::formationMsg>("/formation", 10);
  algParam_pub=nh.advertise<enoga::algMsg>("/algorithm_params", 10);
  
  initFormations();
  updateTopicList();
  ui_.leaderBox->setCurrentIndex(ui_.leaderBox->findText(""));
  connect(ui_.leaderBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.progressBar_4->setValue(0);
  //ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  //connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  //ui_.save_as_image_push_button->setIcon(QIcon::fromTheme("document-save-as"));
  //connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this, SLOT(saveImage()));

  //connect(ui_.takeoffButton, SIGNAL(pressed()), this, SLOT(onTakeoff()));
  //connect(ui_.landButton, SIGNAL(pressed()), this, SLOT(onLand()));
  connect(ui_.upButton, SIGNAL(pressed()), this, SLOT(onUp()));
  connect(ui_.downButton, SIGNAL(pressed()), this, SLOT(onDown()));
  connect(ui_.yawLeftButton, SIGNAL(pressed()), this, SLOT(onYawL()));
  connect(ui_.yawRightButton, SIGNAL(pressed()), this, SLOT(onYawR()));
  connect(ui_.forwardButton, SIGNAL(pressed()), this, SLOT(onFrw()));
  connect(ui_.backwardButton, SIGNAL(pressed()), this, SLOT(onBack()));
  connect(ui_.rotRightButton, SIGNAL(pressed()), this, SLOT(onRollR()));
  connect(ui_.rotLeftButton, SIGNAL(pressed()), this, SLOT(onRollL()));
  connect(ui_.armButton, SIGNAL(pressed()), this, SLOT(onArm()));
  connect(ui_.armButton_2, SIGNAL(pressed()), this, SLOT(onArm()));
  connect(ui_.offboardOnButton, SIGNAL(pressed()), this, SLOT(onOffboardOn()));
  connect(ui_.offboardOnButton_2, SIGNAL(pressed()), this, SLOT(onOffboardOn()));
  connect(ui_.setFormationButton, SIGNAL(pressed()), this, SLOT(setFormation()));
  connect(ui_.plusGradusRotFormButton, SIGNAL(pressed()), this, SLOT(plusGradus()));
  connect(ui_.minusGradusRotFormButton, SIGNAL(pressed()), this, SLOT(minusGradus()));
  connect(ui_.algorithmBox_2, SIGNAL(currentIndexChanged(QString)), this, SLOT(refreshForm(QString)));
  connect(ui_.formParamBox, SIGNAL(valueChanged(double)), this, SLOT(updateFormParameter()));
  connect(ui_.formSecAngBox, SIGNAL(valueChanged(double)), this, SLOT(updateFormSecAngle()));
  connect(ui_.helpFormButton, SIGNAL(pressed()), this, SLOT(showFormationHelpMessage()));
  connect(ui_.addButton_3, SIGNAL(pressed()), this, SLOT(onAddCheckpointPressed()));
  connect(ui_.startButton_3, SIGNAL(pressed()), this, SLOT(onStartPressed()));
  connect(ui_.homeButton_3, SIGNAL(pressed()), this, SLOT(onHomePressed()));
  connect(ui_.endButton, SIGNAL(pressed()), this, SLOT(onHomePressed()));
   //connect(ui_.autortl, SIGNAL(pressed()), this, SLOT(autortl()));
   //connect(ui_.autoland, SIGNAL(pressed()), this, SLOT(autoland()));
   //connect(ui_.connect, SIGNAL(pressed()), this, SLOT(connect()));
  //connect(ui_.disconnect, SIGNAL(pressed()), this, SLOT(disconnect()));
  //connect(ui_.saveButton, SIGNAL(pressed()), this, SLOT(controlSettingsSave));
  //connect(ui_.loadButton, SIGNAL(pressed()), this, SLOT(controlSettingsLoad));
  connect(ui_.saveButton, &QPushButton::clicked, this, &rqt_uav_gui_cpp::GUIPlugin::controlSettingsSave);
  connect(ui_.loadButton, &QPushButton::clicked, this, &rqt_uav_gui_cpp::GUIPlugin::controlSettingsLoad);
  connect(ui_.addButton_4, SIGNAL(pressed()), this, SLOT(refreshAlgorithm()));
  
  //TODO: старая попытка сделать прикольный индикатор батареи
  //ui_.Slider_2->show();
  formationGraphics=ui_.formationGraphics;
  ui_.helpFormButton->setToolTip(tr("Opens help menu"));
  ui_.plusGradusRotFormButton->setToolTip(tr("Rotates the formation 45 degrees counterclockwise"));
  ui_.minusGradusRotFormButton->setToolTip(tr("Rotates the formation 45 degrees clockwise"));
  ui_.setFormationButton->setToolTip(tr("Applies the formation settings and sends data to the leader"));
  
  connect(ui_.progressBatteryBar, SIGNAL(valueChanged(int)), this, SLOT(batteryLevelChanged(int)));
//  ui_.progressBatteryBar->setValue(100);
/*  ui_.upButton->setStyleSheet("border-image:url(:/imgs/up.svg);");
  ui_.downButton->setStyleSheet("border-image:url(:/imgs/down.svg);");
  ui_.endButton->setStyleSheet("border-image:url(:/imgs/home.svg);");
  ui_.backwardButton->setStyleSheet("border-image:url(:/imgs/arrow_back.svg);");
  ui_.forwardButton->setStyleSheet("border-image:url(:/imgs/arrow_forward.svg);");
  ui_.yawLeftButton->setStyleSheet("border-image:url(:/imgs/arrow_left.svg);");
  ui_.yawRightButton->setStyleSheet("border-image:url(:/imgs/arrow_right.svg);");
  ui_.rotLeftButton->setStyleSheet("border-image:url(:/imgs/90left.svg);");
  ui_.rotRightButton->setStyleSheet("border-image:url(:/imgs/90right.svg);");*/
  
  
  
  //scene=new QGraphicsScene(this);
  //view=new QGraphicsView();
  //ui_.tabWidget->widget(2)->formationGV;
  //view=ui_.tabWidget->widget(2)->findChild<QGraphicsView *>("formationGV");
  
  //qWarning("%d", view);
  //view=ui_.tabWidget->widget(3)->findChild<QGraphicsView *>("formationGV");
  //qWarning("%d", view);
  //view=widget_->findChild<QGraphicsView *>("formationGV");
  //qWarning("%d", view);
  //view->setScene(scene);
  //QPointF *point=new QPointF(4.43, 5.11);
  //QPointF *point1=new QPointF(-4.43, -8.003);
  //scene->addEllipse(point->x(), point->y(), 5, 5);
  //scene->addEllipse(point1->x(), point1->y(), 5, 5);
  //view->show();
  
  ui_.downCameraFrame->setOuterLayout(ui_.downCameraLayout);
  ui_.downCameraFrame_4->setOuterLayout(ui_.downCameraLayout);
  ui_.forwardCameraFrame_4->setOuterLayout(ui_.downCameraLayout);
  ui_.forwardCameraFrame_2->setOuterLayout(ui_.downCameraLayout);
  ui_.algorithmBox->addItem("Leader Followers"); 
  ui_.algorithmBox->addItem("VBRAC"); 
//  ui_.imageFrameOF->setOuterLayout(ui_.imageLayoutOF);

  QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?");
}

void GUIPlugin::shutdownPlugin() {
  subscriber_.shutdown();
  forw_subscriber_.shutdown();
//  subscriber2_.shutdown();
  pub.shutdown();
}

void GUIPlugin::controlSettingsSave() { 
  QSettings settings("LIRS","settingsGUI");
  settings.setValue("minHeigh", ui_.longitudeBox_6->value());
  settings.setValue("maxHeigh", ui_.latitudeBox_6->value());
  settings.setValue("parameter", ui_.parameterBox->value());
  settings.setValue("battery", ui_.batteryBox->value());
  settings.setValue("algorithm", ui_.algorithmBox->currentIndex());
}


void GUIPlugin::controlSettingsLoad() {
  QSettings settings("LIRS","settingsGUI");
  ui_.longitudeBox_6->setValue(settings.value("minHeigh").toDouble());
  ui_.latitudeBox_6->setValue(settings.value("maxHeigh").toDouble());
  ui_.parameterBox->setValue(settings.value("parameter").toDouble());
  ui_.batteryBox->setValue(settings.value("battery").toDouble());
  ui_.algorithmBox->setCurrentIndex(settings.value("algorithm").toInt());
}

/*
void GUIPlugin::onLand() {
  std_msgs::Char msg;
  //msg.data = 'h'; //k
  msg.data = 'g';
  pub.publish(msg);
}

void GUIPlugin::onTakeoff() {
  std_msgs::Char msg;
  msg.data = 't';
  pub.publish(msg);
}
*/

void GUIPlugin::onArm() {  
    std_msgs::Int8 msg;
    msg.data = 1;
    pub.publish(msg);
    armPressed=true;
}

void GUIPlugin::onOffboardOn() { 
    std_msgs::Int8 msg;
    msg.data = 2;
    pub.publish(msg);
    msg.data = -5;
    pub.publish(msg);
  }

void GUIPlugin::onUp() {
    std_msgs::Int8 msg;
    msg.data = 3;
    pub.publish(msg);
  }

void GUIPlugin::onDown() {
    std_msgs::Int8 msg;
    msg.data = 4;
    pub.publish(msg);
  }

void GUIPlugin::onYawL() {
    std_msgs::Int8 msg;
    msg.data = 5;
    pub.publish(msg);
  }

void GUIPlugin::onYawR() {
    std_msgs::Int8 msg;
    msg.data = 6;
    pub.publish(msg);
  }

void GUIPlugin::onFrw() {
    std_msgs::Int8 msg;
    msg.data = 7;
    pub.publish(msg);
  }
void GUIPlugin::onBack() {
    std_msgs::Int8 msg;
    msg.data = 8;
    pub.publish(msg);
  }
void GUIPlugin::onRollL() {
    std_msgs::Int8 msg;
    msg.data = 9;
    pub.publish(msg);
  }

void GUIPlugin::onRollR() {
    std_msgs::Int8 msg;
    msg.data = 10;
    pub.publish(msg);
  }



void GUIPlugin::autoland() {
    std_msgs::Int8 msg;
    msg.data = -2;
    pub.publish(msg);
}

void GUIPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  //QString topic = ui_.leaderBox->currentText();
  //instance_settings.setValue("topic", topic);
//  QString topic2 = ui_.observerBox->currentText();
//  instance_settings.setValue("topic2", topic2);
}

void GUIPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  QString topic = instance_settings.value("topic", "").toString();
  selectTopic(topic);
//  QString topic2 = instance_settings.value("topic2", "").toString();
//  selectTopic2(topic2);
}

//-----------------------------------------------------------------

struct Point {
	float x;
	float y;
  float z;
};

std::vector<Point> checkpoints_;

void GUIPlugin::onAddCheckpointPressed() {

  bool okLon, okLat, okAlt;
  double longitude = ui_.longitudeBox_3->value(); 
  double latitude = ui_.latitudeBox_3->value();
  double altitude = ui_.altitudeBox_3->value(); 

  
  Point point;
  point.x = longitude;
  point.y = latitude;
  point.z = altitude;
  checkpoints_.push_back(point);
  k = k+1;
  
  ui_.tableWidget_2->insertRow(ui_.tableWidget_2->rowCount());
  ui_.tableWidget_2->setColumnCount(3);
  QStringList horizontalHeaders;
  horizontalHeaders << "x" << "y"<< "z";
  ui_.tableWidget_2->setColumnCount(horizontalHeaders.size());
  ui_.tableWidget_2->setHorizontalHeaderLabels(horizontalHeaders);
  ui_.tableWidget_2->setItem(ui_.tableWidget_2->rowCount()-1, 0, new QTableWidgetItem(QString::number(longitude)));
  ui_.tableWidget_2->setItem(ui_.tableWidget_2->rowCount()-1, 1, new QTableWidgetItem(QString::number(latitude)));
  ui_.tableWidget_2->setItem(ui_.tableWidget_2->rowCount()-1, 2, new QTableWidgetItem(QString::number(altitude)));
  
}

bool flag = false;

void GUIPlugin::onStartPressed() {
  QString pointsStr = "Sending checkpoints:\n";
  waypoints_pub=nh.advertise<enoga::points>("/coordinates_for_leader", 10);
  for (const auto& point : checkpoints_) {
    pointsStr += QString("Longitude: %1, Latitude: %2, Altitude: %3\n")
                     .arg(point.x, 0, 'f', 6) 
                     .arg(point.y, 0, 'f', 6)
                     .arg(point.z, 0, 'f', 6);   
  }
  for(std::vector<Point>::iterator it=checkpoints_.begin(); it!=checkpoints_.end(); ++it) {
  	geometry_msgs::Point p;
  	p.x=(*it).x;
  	p.y=(*it).y;
  	p.z=(*it).z;
  	points_msg.points.push_back(p);
    flag = true;
  }


  // Показать сообщение с точками
  QMessageBox::information(widget_, "Checkpoints to Send", pointsStr);

  waypoints_pub.publish(points_msg);
  std_msgs::Int8 msg;
  msg.data = -8;
  pub.publish(msg);
  checkpoints_.clear();
  fl = 1;
}

//mavros_msgs::State st;
enoga::guiMas inf;


void GUIPlugin::gui_msgCallback(const enoga::guiMas& msg){
  bat_v = msg.batteries;
  stat_v = msg.states;
}

void GUIPlugin::groups_Callback(const enoga::groups& msg){
  outer_ids=msg.outer;
  inner_ids=msg.inner;
  reserve_ids=msg.reserve;
}

void GUIPlugin::updateTable() {
  //ВАЖНО: leader_id заменить на переменную
  int col = numberOfUavs-1;
  int leader_id = 0;
  bool statusIsOk;
  ui_.tableWidget->setRowCount(col);
  ui_.tableWidget->setColumnCount(4);
  QStringList horizontalHeaders;
  horizontalHeaders << "id" << "type"<< "battery" << "status";
  ui_.tableWidget->setColumnCount(horizontalHeaders.size());
  ui_.tableWidget->setHorizontalHeaderLabels(horizontalHeaders);
  if (armPressed){
    for (int row = 0; row < col; ++row) {
      ui_.tableWidget->setItem(row, 0, new QTableWidgetItem("uav_"+QString::number(row)));
      if(ui_.algorithmBox->currentText()==QString("VBRAC")) {
        groups_sub = nh.subscribe("/groups", 10, &GUIPlugin::groups_Callback, this);
        if(std::binary_search(outer_ids.begin(), outer_ids.end(), row)) {
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("outer"));
        }
        if(std::binary_search(inner_ids.begin(), inner_ids.end(), row)) {
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("inner"));
        }
        if(std::binary_search(reserve_ids.begin(), reserve_ids.end(), row)) {
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("reserve"));
        }
        if(row==leader_id){
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("leader"));
        }

      }
      else {
        if(row==leader_id){
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("leader"));
        }
        else {
          ui_.tableWidget->setItem(row, 1, new QTableWidgetItem("follower"));
        }
      }

      gui_sub = nh.subscribe("/gui_info", 10, &GUIPlugin::gui_msgCallback, this);
      for(std::vector<float>::iterator i = bat_v.begin(); i!=bat_v.end(); ++i){
        int index_bat = std::distance(bat_v.begin(), i);
        if(index_bat == row){
          if(index_bat==leader_id)
          	batteryLevelChanged(bat_v[leader_id]);
          ui_.tableWidget->setItem(row, 2, new QTableWidgetItem(QString::number(*i)+"%"));
        }
      }

      for(std::vector<float>::iterator y = stat_v.begin(); y!=stat_v.end();++y){
        int index_stat = std::distance(stat_v.begin(), y);
        if(index_stat == row){
          if(*y == 1){
            ui_.tableWidget->setItem(row, 3, new QTableWidgetItem("OK"));
          }
          else {
            ui_.tableWidget->setItem(row, 3, new QTableWidgetItem("DISABLED"));
            ui_.tableWidget->setItem(row, 2, new QTableWidgetItem("None"));
          }
        }
      }
    }
  }  
}

double pr = 0.;

void destinationCall (const std_msgs::Int8 msg) {
  pr = pr;
}


void GUIPlugin::updateProgressBar() {
  destination_check=nh.subscribe("/destination_signal", 10, &destinationCall);
  double progress = (pr/k)*100;
  ui_.progressBar_4->setValue(progress);
}

void GUIPlugin::onStopPressed() {
    std_msgs::Int8 msg;
    // msg.data = -9; 
    // pub.publish(msg);
}

void GUIPlugin::onHomePressed() {
    std_msgs::Int8 msg;
    msg.data = -2;
    pub.publish(msg);
}


//--------------------------------------------------------------------

void GUIPlugin::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  //message_types.insert("sensor_msgs/CompressedImage");
  //QSet<QString> message_sub_types;
  //message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.leaderBox->currentText();
//  QString selected2 = ui_.observerBox->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, /*message_sub_types,*/ transports).values();
  topics.append("");
  qSort(topics);
  ui_.leaderBox->clear();
//  ui_.observerBox->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    if (label.contains("forward")) {
      topicsfw.append(label);
    }
    else {
      ui_.leaderBox->addItem(label, QVariant(*it));
      numberOfUavs +=1;
    }
    //ui_.observerBox->addItem(label, QVariant(*it));
  }
  form.setNumber(numberOfUavs-1);
  refreshForm(QString(""));

  // restore previous selection
  selectTopic(selected);
//  selectTopic2(selected2);
}

QSet<QString> GUIPlugin::getTopics(const QSet<QString>& message_types, /*const QSet<QString>& message_sub_types,*/ const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.insert(topic);

      // add transport specific sub-topics
      /*for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
        }
      }*/
    }
    /*if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
      }
    }*/
  }
  return topics;
}

void GUIPlugin::selectTopic(const QString& topic)
{
  int index = ui_.leaderBox->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.leaderBox->addItem(label, QVariant(topic));
    index = ui_.leaderBox->findText(topic);
  }
  ui_.leaderBox->setCurrentIndex(index);
}

void GUIPlugin::onTopicChanged(int index)
{
  conversion_mat_.release();

  subscriber_.shutdown();

  // reset image on topic change
  ui_.downCameraFrame->setImage(QImage());
  ui_.downCameraFrame_4->setImage(QImage());

  QStringList parts = ui_.leaderBox->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";
  //ui_.leaderText->setText(topic);

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it.subscribe(topic.toStdString(), 1, &GUIPlugin::callbackImage, this, hints);
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }

  forw_conversion_mat_.release();

  forw_subscriber_.shutdown();

  ui_.forwardCameraFrame_4->setImage(QImage());
  ui_.forwardCameraFrame_2->setImage(QImage());
  QString topicfw = topicsfw[index-1];
  QStringList parts_forw = topicfw.split(" ");
  QString topic_forw = parts_forw.first();
  QString transport_forw = parts_forw.length() == 2 ? parts_forw.last() : "raw";

  if (!topic_forw.isEmpty())
  {
    image_transport::ImageTransport it_forw(getNodeHandle());
    image_transport::TransportHints hints_forw(transport_forw.toStdString());
    try {
      forw_subscriber_ = it_forw.subscribe(topic_forw.toStdString(), 1, &GUIPlugin::callbackForwImage, this, hints_forw);
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

/*
void GUIPlugin::saveImage()
{
  // take a snapshot before asking for the filename
  QImage img = ui_.imageFrameLF->getImageCopy();

  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty()) return;
  img.save(file_name);
}
*/

/*
void GUIPlugin::invertPixels(int x, int y)
{
  cv::Vec3b & pixel = conversion_mat_.at<cv::Vec3b>(cv::Point(x, y));
  if (pixel[0] + pixel[1] + pixel[2] > 3 * 127)
    pixel = cv::Vec3b(0,0,0);
  else
    pixel = cv::Vec3b(255,255,255);
}
*/

void GUIPlugin::callbackForwImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    forw_conversion_mat_ = cv_ptr->image;
    if (forw_conversion_mat_.data) {
      bin_mat_ = cv::Mat(forw_conversion_mat_.size(), CV_8UC1);
      for (int i = 0; i < forw_conversion_mat_.rows; i++) {
        for (int j = 0; j < forw_conversion_mat_.cols; j++) {
          bin_mat_.at<uchar>(i, j) = (forw_conversion_mat_.at<cv::Vec3b>(i, j)[2] > forw_conversion_mat_.at<cv::Vec3b>(i, j)[1] && forw_conversion_mat_.at<cv::Vec3b>(i, j)[2] > forw_conversion_mat_.at<cv::Vec3b>(i, j)[0])
                  ? (forw_conversion_mat_.at<cv::Vec3b>(i, j)[2] * 2 - (forw_conversion_mat_.at<cv::Vec3b>(i, j)[1] + forw_conversion_mat_.at<cv::Vec3b>(i, j)[0])) / 2 : 0;
          //bin_mat_.at<uchar>(i, j) = 255;
        }
      }
      cv::threshold(bin_mat_, bin_mat_, 10, 255, cv::THRESH_BINARY);
      cv::Moments m = cv::moments(bin_mat_, true);
      cv::Point p(m.m10/m.m00, m.m01/m.m00);
      cv::circle(forw_conversion_mat_, p, 5, cv::Scalar(255, 255, 0), -1);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        forw_conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, forw_conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("GUIPlugin.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        ui_.forwardCameraFrame_4->setImage(QImage());
        ui_.forwardCameraFrame_2->setImage(QImage());
        //ui_.downCameraFrame_4->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("GUIPlugin.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      ui_.forwardCameraFrame_4->setImage(QImage());
      ui_.forwardCameraFrame_2->setImage(QImage());
      //ui_.downCameraFrame_4->setImage(QImage());
      return;
    }
  }
  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(forw_conversion_mat_.data, forw_conversion_mat_.cols, forw_conversion_mat_.rows, forw_conversion_mat_.step[0], QImage::Format_RGB888);
  ui_.forwardCameraFrame_4->setImage(image);
  ui_.forwardCameraFrame_2->setImage(image);
  //ui_.downCameraFrame_4->setImage(image);
}

void GUIPlugin::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
    if (conversion_mat_.data) {
      bin_mat_ = cv::Mat(conversion_mat_.size(), CV_8UC1);
      for (int i = 0; i < conversion_mat_.rows; i++) {
        for (int j = 0; j < conversion_mat_.cols; j++) {
          bin_mat_.at<uchar>(i, j) = (conversion_mat_.at<cv::Vec3b>(i, j)[2] > conversion_mat_.at<cv::Vec3b>(i, j)[1] && conversion_mat_.at<cv::Vec3b>(i, j)[2] > conversion_mat_.at<cv::Vec3b>(i, j)[0])
                  ? (conversion_mat_.at<cv::Vec3b>(i, j)[2] * 2 - (conversion_mat_.at<cv::Vec3b>(i, j)[1] + conversion_mat_.at<cv::Vec3b>(i, j)[0])) / 2 : 0;
          //bin_mat_.at<uchar>(i, j) = 255;
        }
      }
      cv::threshold(bin_mat_, bin_mat_, 10, 255, cv::THRESH_BINARY);
      cv::Moments m = cv::moments(bin_mat_, true);
      cv::Point p(m.m10/m.m00, m.m01/m.m00);
      cv::circle(conversion_mat_, p, 5, cv::Scalar(255, 255, 0), -1);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("GUIPlugin.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        ui_.downCameraFrame->setImage(QImage());
        ui_.downCameraFrame_4->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("GUIPlugin.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      ui_.downCameraFrame->setImage(QImage());
      ui_.downCameraFrame_4->setImage(QImage());
      return;
    }
  }
  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
  ui_.downCameraFrame->setImage(image);
  ui_.downCameraFrame_4->setImage(image);
}

//инициализация списка доступных формаций
void GUIPlugin::initFormations() {
	QStringList availableForms=QStringList()<<"Wedge"<<"Wedge with first UAV"<<"Snake"<<"Front"<<"Column"<<"Echelon"<<"Flanks"<<"Chess"<<"Rectangle"<<"Rectangle (empty inside)"<<"Rhombus"<<"Rhombus (empty inside)"<<"Sector of a circle"<<"Circle";
	ui_.algorithmBox_2->addItems(availableForms);
	form.setNumber(5);
	form.setParameter(5+5);
	form.setSectorAngle(90);
	form.setArcLength(30+5);
	form.generate_points_klin();
	formCmd.form=0;
	formCmd.parameter=10;
	formCmd.secangle=90;
	formCmd.arclength=35;
	formationScene=new QGraphicsScene();
	ui_.formationGraphics->setScene(formationScene);
	std::vector<std::vector<double>> points=form.getPoints();
	//QMessageBox msgBox;
	//msgBox.setText("The parameter specifies the distance between the UAVs in XY axis.\nSector angle and arc length do not used in wedge formation.");
	//msgBox.exec();
	for(size_t i=0; i<points.size(); ++i) {
		
		QwtPlotMarker *points_qwt=new QwtPlotMarker();
    	points_qwt->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, Qt::blue, Qt::NoPen, QSize(5, 5)));
    	points_qwt->setXValue(points[i][0]);
	    points_qwt->setYValue(points[i][1]);
    	points_qwt->attach(ui_.formPlot);
    
		//qWarning("%d %f %f", i, points[i][0], points[i][1]);
	}
    
    ui_.formPlot->setAxisScale(QwtPlot::xBottom, -(form.getNumber()*(form.getParameter()+5))*2, form.getNumber()*(form.getParameter()+5)*2); //Размер определяется по кол-во БЛА*(параметр+5)
    ui_.formPlot->setAxisScale(QwtPlot::yLeft, -(form.getNumber()*(form.getParameter()+5))*2, form.getNumber()*(form.getParameter()+5)*2);
    
    ui_.formPlot->setAxisTitle(QwtPlot::xBottom, "X");
    ui_.formPlot->setAxisTitle(QwtPlot::yLeft, "Y");

    // Отображение осей
    QwtPlotMarker *xAxis_qwt = new QwtPlotMarker();
    xAxis_qwt->setLineStyle(QwtPlotMarker::HLine);
    xAxis_qwt->setLinePen(Qt::red, 0, Qt::DashLine);
    xAxis_qwt->setYValue(0);
    xAxis_qwt->attach(ui_.formPlot);

    QwtPlotMarker *yAxis_qwt = new QwtPlotMarker();
    yAxis_qwt->setLineStyle(QwtPlotMarker::VLine);
    yAxis_qwt->setLinePen(Qt::green, 0, Qt::DashLine);
    yAxis_qwt->setXValue(0);
    yAxis_qwt->attach(ui_.formPlot);
}

//изменение градуса поворота формации +45 градусов
void GUIPlugin::plusGradus() {
	angle=45;
	//form.changeRotationAngle(angle);
	form.setFormRotAngle(angle);
	//formationScene->clear();
	//std::vector<std::vector<double>> points=form.getPoints();
	//for(size_t i=0; i<points.size(); ++i) {
	//	formationScene->addEllipse(points[i][0], points[i][1], 5, 5, QPen(Qt::black), QBrush(Qt::blue));
	//}
	refreshForm(QString(""));
}

//изменение градуса поворота формации -45 градусов
void GUIPlugin::minusGradus() {
	angle=-45;
	//form.changeRotationAngle(angle);
	form.setFormRotAngle(angle);
	//formationScene->clear();
	//std::vector<std::vector<double>> points=form.getPoints();
	//for(size_t i=0; i<points.size(); ++i) {
	//	formationScene->addEllipse(points[i][0], points[i][1], 5, 5, QPen(Qt::black), QBrush(Qt::blue));
	//}
	refreshForm(QString(""));
}

//обновление изображения формации, отрисовка в окне, вызывается при изменении выбранного построения или его параметров
void GUIPlugin::refreshForm(QString text) {
	//qWarning("%s", qUtf8Printable(ui_.algorithmBox_2->currentText()));
	if(ui_.algorithmBox_2->currentText()==QString("Wedge")) {
		form.generate_points_klin();
		formCmd.form=0;
	} else if(ui_.algorithmBox_2->currentText()==QString("Wedge with first UAV")) {
		form.generate_points_klin_with_first();
		formCmd.form=1;
	} else if(ui_.algorithmBox_2->currentText()==QString("Snake")) {
		form.generate_points_snake();
		formCmd.form=2;
	} else if(ui_.algorithmBox_2->currentText()==QString("Front")) {
		form.generate_points_front();
		formCmd.form=3;
	} else if(ui_.algorithmBox_2->currentText()==QString("Column")) {
		form.generate_points_column();
		formCmd.form=4;
	} else if(ui_.algorithmBox_2->currentText()==QString("Circle")) {
		form.generate_points_circle();
		formCmd.form=13;
	} else if(ui_.algorithmBox_2->currentText()==QString("Echelon")) {
		form.generate_points_peleng();
		formCmd.form=5;
	} else if(ui_.algorithmBox_2->currentText()==QString("Flanks")) {
		form.generate_points_flangs();
		formCmd.form=6;
	} else if(ui_.algorithmBox_2->currentText()==QString("Chess")) {
		form.generate_points_chess();
		formCmd.form=7;
	} else if(ui_.algorithmBox_2->currentText()==QString("Rectangle")) {
		form.generate_points_rect_full();
		formCmd.form=8;
	} else if(ui_.algorithmBox_2->currentText()==QString("Rectangle (empty inside)")) {
		form.generate_points_rect_empty();
		formCmd.form=9;
	} else if(ui_.algorithmBox_2->currentText()==QString("Rhombus")) {
		form.generate_points_romb();
		formCmd.form=10;
	} else if(ui_.algorithmBox_2->currentText()==QString("Rhombus (empty inside)")) {
		form.generate_points_romb_empty();
		formCmd.form=11;
	} else if(ui_.algorithmBox_2->currentText()==QString("Sector of a circle")) {
		form.generate_points_sector();
		formCmd.form=12;
	}
	
	//form.changeRotationAngle(0);
	//formCmd.formrotangle=angle;
	std::vector<std::vector<double>> points=form.getPoints();
	
	ui_.formPlot->detachItems(QwtPlotItem::Rtti_PlotItem);
	for(size_t i=0; i<points.size(); ++i) {
		
		QwtPlotMarker *points_qwt=new QwtPlotMarker();
    	points_qwt->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, Qt::blue, Qt::NoPen, QSize(5, 5)));
    	points_qwt->setXValue(points[i][0]);
	    points_qwt->setYValue(points[i][1]);
    	points_qwt->attach(ui_.formPlot);
    	
		qWarning("%d %lf %lf", i, points[i][0], points[i][1]);
	}
    
    ui_.formPlot->setAxisScale(QwtPlot::xBottom, -(form.getNumber()*(form.getParameter()+5))*2, form.getNumber()*(form.getParameter()+5)*2); //Размер определяется по кол-во БЛА*(параметр+5)
    ui_.formPlot->setAxisScale(QwtPlot::yLeft, -(form.getNumber()*(form.getParameter()+5))*2, form.getNumber()*(form.getParameter()+5)*2);
    
    ui_.formPlot->setAxisTitle(QwtPlot::xBottom, "X");
    ui_.formPlot->setAxisTitle(QwtPlot::yLeft, "Y");

    // Отображение осей
    QwtPlotMarker *xAxis_qwt = new QwtPlotMarker();
    xAxis_qwt->setLineStyle(QwtPlotMarker::HLine);
    xAxis_qwt->setLinePen(Qt::red, 0, Qt::DashLine);
    xAxis_qwt->setYValue(0);
    xAxis_qwt->attach(ui_.formPlot);

    QwtPlotMarker *yAxis_qwt = new QwtPlotMarker();
    yAxis_qwt->setLineStyle(QwtPlotMarker::VLine);
    yAxis_qwt->setLinePen(Qt::green, 0, Qt::DashLine);
    yAxis_qwt->setXValue(0);
    yAxis_qwt->attach(ui_.formPlot);
}

//обновление параметра формации, вызывается при изменении параметра
void GUIPlugin::updateFormParameter() {
	form.setParameter(ui_.formParamBox->value());
	refreshForm(QString(""));
	formCmd.parameter=ui_.formParamBox->value();
	//qWarning("%f", ui_.formParamBox->value());
}

//обновление угла сектора формации, вызывается при измнении угла сектора формации
void GUIPlugin::updateFormSecAngle() {
	form.setSectorAngle(ui_.formSecAngBox->value());
	refreshForm(QString(""));
	formCmd.secangle=ui_.formSecAngBox->value();
	//qWarning("%f", ui_.formSecAngBox->value());
}

//установка выбранной и настроенной формации (отправка информации и команды лидеру)
void GUIPlugin::setFormation() {
	formCmd.formrotangle=form.getRotationAngle();//angle;//form.getRotationAngle();
	qWarning("%f", formCmd.formrotangle);
	formPub.publish(formCmd);
}

//выдача сообщения с сообщением-помощью по формациям
void GUIPlugin::showFormationHelpMessage() {
	QMessageBox msgBox;
	msgBox.setWindowTitle("Formations Help");
	if(ui_.algorithmBox_2->currentText()==QString("Wedge")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/wedge.png></div><br>This formation has a form of wedge, which has a triangular shape. Parameter is a distance between the UAV along the X and Y axes. Therefore, the distance between neighboring UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Wedge with first UAV")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/wedgeWithFirst.png></div><br>This formation has a form of wedge, which has a triangular shape, but the first UAV stands out in front of the formation, as if leading a swarm. Parameter is a distance between the UAV along the X and Y axes. Therefore, the distance between neighboring UAVs is equal to the &#8730;2*Parameter. The exception is the distance between the first and second UAVs of the swarm, it is equal to the Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Snake")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/snake.png></div><br>This formation has a form of snake and has only two columns. The distance between UAVs in one column is equal to the 2*Parameter, and the distance between the first and the second columns along the X and Y axes is equal to the Parameter, so the distance between two UAVs from different columns is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Front")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/front.png></div><br>This formation has a form of one line. The distance between UAVs is equal to the Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Column")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/column.png></div><br>This formation has a form of one column. The distance between UAVs is equal to the Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Circle")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/circle.png></div><br>This formation has a form of circle. The operator sets the Parameter, which is a radius of the circle.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Echelon")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/peleng.png></div><br>This formation has a form of diagonal line to the right. The distance between UAVs along the X and Y axes is equal to the Parameter, so the distance between two UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Flanks")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/flanks.png></div><br>This formation has a form of two diagonal lines to the right and to the left sides. It is called right and left wings. The distance between neighboring UAVs along the X and Y axes is equal to the Parameter, so the distance between two neighboring UAVs is equal to the &#8730;2*Parameter. The exception is the distance between the first UAVs of the right and left wings, it is equal to the Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Chess")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/chess.png></div><br>This formation has a form of chess. The distance between UAVs in one column or line is equal to the 2*Parameter. And the distance between two neighboring UAVs from different columns and lines along the X and Y axes is equal to the Parameter, so the distance between two neighboring UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Rectangle")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/rectangleFull.png></div><br>This formation has a form of rectangle filled inside. The distance between UAVs in one column or line is equal to the Parameter. And the distance between two neighboring UAVs from different columns and lines along the X and Y axes is equal to the Parameter, so the distance between two neighboring UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Rectangle (empty inside)")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/rectangleEmpty.png></div><br>This formation has a form of rectangle empty inside. The distance between UAVs in one column or line is equal to the Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Rhombus")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/rhombusFull.png></div><br>This formation has a form of rhombus filled inside. The distance between UAVs in one column or line is equal to the Parameter. And the distance between two neighboring UAVs from different columns and lines along the X and Y axes is equal to the Parameter, so the distance between two neighboring UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Rhombus (empty inside)")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/rhombusEmpty.png></div><br>This formation has a form of rhombus empty inside. The distance between two neighboring UAVs along the X and Y axes is equal to the Parameter, so the distance between two neighboring UAVs is equal to the &#8730;2*Parameter.");
	} else if(ui_.algorithmBox_2->currentText()==QString("Sector of a circle")) {
		msgBox.setText("<div style=\"text-align: center;\"><img src=:/imgs/sector.png></div><br>This formation has a form of sector of the circle. The operator sets the Parameter, which is a radius of the sector of the circle, and the angle of the sector of the circle, then the arc length calculated automatically. Next, the UAVs distributed evenly along the formed arc.");
	}
	msgBox.setStyleSheet("background-color: white;");
	msgBox.exec();
}

void GUIPlugin::batteryLevelChanged(int nValue) {
	ui_.progressBatteryBar->setValue(nValue);
	QString myStyleSheet = " QProgressBar::chunk {"
	" background-color: ";
	if(nValue<=20) {
    		myStyleSheet.append("red;");
    	} else if(nValue<=70&&nValue>40) {
    		myStyleSheet.append("yellow;");
	} else if(nValue<=40&&nValue>20) {
    		myStyleSheet.append("orange;");
	} else {
    		myStyleSheet.append("lightgreen;");
	}
	myStyleSheet.append("     width: 10px;"\
			     "     margin: 0.5px;"\
                    	     " }");
        if(nValue<=20) {
    		myStyleSheet.append("QProgressBar { color: black; }");
    	} else if(nValue<=70&&nValue>40) {
    		myStyleSheet.append("QProgressBar { color: black; }");
	} else if(nValue<=40&&nValue>20) {
    		myStyleSheet.append("QProgressBar { color: black; }");
	} else {
    		myStyleSheet.append("QProgressBar { color: cyan; }");
	}
	myStyleSheet.append("QProgressBar { color: black; }");
	ui_.progressBatteryBar->setStyleSheet(myStyleSheet);
}

//обновление выбранного алгоритма управления роем, отправка параметров
void GUIPlugin::refreshAlgorithm() {
	if(ui_.algorithmBox->currentText()==QString("Leader Followers")) {
		algDet.type=0;
	} else if(ui_.algorithmBox->currentText()==QString("VBRAC")) {
		algDet.type=1;
	}
	algDet.minH=ui_.longitudeBox_6->value();
	algDet.maxH=ui_.latitudeBox_6->value();
	algDet.landBat=ui_.batteryBox->value();
	algDet.param=ui_.parameterBox->value();
	//qWarning("%d %d %d %f %f", algDet.type, algDet.minH, algDet.maxH, algDet.landBat, algDet.param);
	//TODO: отправка сообщения лидеру с обновленными параметрами
	enoga::algMsg msg;
	msg.minH=algDet.minH;
	msg.maxH=algDet.maxH;
	msg.type=algDet.type;
	msg.landBat=algDet.landBat;
	msg.param=algDet.param;
  msg.numb=numberOfUavs;
	algParam_pub.publish(msg);
}
}  // namespace
PLUGINLIB_EXPORT_CLASS(rqt_uav_gui_cpp::GUIPlugin, rqt_gui_cpp::Plugin)
