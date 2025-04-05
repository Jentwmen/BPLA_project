#include "rqt_uav_gui_cpp/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMessageBox>
#include <QPainter>

namespace rqt_uav_gui_cpp
{
    GUIPlugin::GUIPlugin()
        : rqt_gui_cpp::Plugin()
        , widget_(0)
    {
        setObjectName("UAV GUI");
    }

    void GUIPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        widget_ = new QWidget();
        ui_.setupUi(widget_);
        context.addWidget(widget_);

        ros::Publisher pub = nh.advertise<std_msgs::Int32>("gui_topic", 10);


        updateTopicList();
        ui_.leaderBox->setCurrentIndex(ui_.leaderBox->findText(""));
        connect(ui_.leaderBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

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
        connect(ui_.offboardOnButton, SIGNAL(pressed()), this, SLOT(onOffboardOn()));

        //connect(ui_.autortl, SIGNAL(pressed()), this, SLOT(autortl()));
        //connect(ui_.autoland, SIGNAL(pressed()), this, SLOT(autoland()));
        //connect(ui_.connect, SIGNAL(pressed()), this, SLOT(connect()));
        //connect(ui_.disconnect, SIGNAL(pressed()), this, SLOT(disconnect()));

        ui_.downCameraFrame->setOuterLayout(ui_.downCameraLayout);
        //  ui_.imageFrameOF->setOuterLayout(ui_.imageLayoutOF);

        QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?");
    }

    void GUIPlugin::shutdownPlugin() {
        subscriber_.shutdown();
        //  subscriber2_.shutdown();
        pub.shutdown();
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
        std_msgs::Int32 msg;
        msg.data = 1;
        pub.publish(msg);
    }

    void GUIPlugin::onOffboardOn() { 
        std_msgs::Int32 msg;
        msg.data = 2;
        pub.publish(msg);
    }

    void GUIPlugin::onUp() {
        std_msgs::Int32 msg;
        msg.data = 3;
        pub.publish(msg);
    }

    void GUIPlugin::onDown() {
        std_msgs::Int32 msg;
        msg.data = 4;
        pub.publish(msg);
    }

    void GUIPlugin::onYawL() {
        std_msgs::Int32 msg;
        msg.data = 5;
        pub.publish(msg);
    }

    void GUIPlugin::onYawR() {
        std_msgs::Int32 msg;
        msg.data = 6;
        pub.publish(msg);
    }

    void GUIPlugin::onFrw() {
        std_msgs::Int32 msg;
        msg.data = 7;
        pub.publish(msg);
    }

    void GUIPlugin::onBack() {
        std_msgs::Int32 msg;
        msg.data = 8;
        pub.publish(msg);
    }

    void GUIPlugin::onRollL() {
        std_msgs::Int32 msg;
        msg.data = 9;
        pub.publish(msg);
    }

    void GUIPlugin::onRollR() {
        std_msgs::Int32 msg;
        msg.data = 10;
        pub.publish(msg);
    }

    /*
    void GUIPlugin::autortl() {
        std_msgs::Int32 msg;
        msg.data = -1;
        pub.publish(msg);
    }
    void GUIPlugin::autoland() {
        std_msgs::Int32 msg;
        msg.data = -2;
        pub.publish(msg);
    }
    void GUIPlugin::connect() {
        std_msgs::Int32 msg;
        msg.data = -5;
        pub.publish(msg);
    }
    void GUIPlugin::disconnect() {
        std_msgs::Int32 msg;
        msg.data = -6;
        pub.publish(msg);
    }
    */


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
            ui_.leaderBox->addItem(label, QVariant(*it));
            //ui_.observerBox->addItem(label, QVariant(*it));
        }

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
            }
            catch (image_transport::TransportLoadException& e) {
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
                cv::Point p(m.m10 / m.m00, m.m01 / m.m00);
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
                }
                else if (msg->encoding == "8UC1") {
                    // convert gray to rgb
                    cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
                }
                else {
                    qWarning("GUIPlugin.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                    ui_.downCameraFrame->setImage(QImage());
                    return;
                }
            }
            catch (cv_bridge::Exception& e)
            {
                qWarning("GUIPlugin.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
                ui_.downCameraFrame->setImage(QImage());
                return;
            }
        }
        // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
        QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
        ui_.downCameraFrame->setImage(image);
    }

}  // namespace
PLUGINLIB_EXPORT_CLASS(rqt_uav_gui_cpp::GUIPlugin, rqt_gui_cpp::Plugin)
