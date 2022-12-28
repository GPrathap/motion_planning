/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include "waypoint_nav_tool.h"

//#include "waypoint_nav_frame.h"
//#include "waypoint_nav_frame.h"

#include <tf/tf.h>

#include <QFileDialog>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

namespace waypoint_nav_plugin
{

WaypointFrame::WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent, WaypointNavTool* wp_tool)
  : QWidget(parent)
  , context_(context)
  , ui_(new Ui::WaypointNavigationWidget())
  , sn_map_ptr_(map_ptr)
  , unique_ind_(unique_ind)
  , server_(server)
  , frame_id_("/map")
  , default_height_(0.0)
  , selected_marker_name_("waypoint1")
  , wp_nav_tool_(wp_tool)
{
  scene_manager_ = context_->getSceneManager();

  // set up the GUI
  ui_->setupUi(this);

  wp_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 1);
  stop_execution_pub_ = nh_.advertise<std_msgs::Empty>("/planning/stop_execution", 1);
  continue_execution_pub_ = nh_.advertise<std_msgs::Empty>("/planning/continue_execution", 1);
  //connect the Qt signals and slots
  connect(ui_->publish_wp_button, SIGNAL(clicked()), this, SLOT(publishButtonClicked()));
  connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this, SLOT(topicChanged()));
  connect(ui_->frame_line_edit, SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(heightChanged(double)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this, SLOT(clearAllWaypoints()));
  connect(ui_->stop_execution_button, SIGNAL(clicked()), this, SLOT(stopExecution()));
  connect(ui_->continue_execution_button, SIGNAL(clicked()), this, SLOT(continueExecution()));

  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));
  connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(poseChanged(double)));

  connect(ui_->save_wp_button, SIGNAL(clicked()), this, SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this, SLOT(loadButtonClicked()));

}

WaypointFrame::~WaypointFrame()
{
  delete ui_;
  sn_map_ptr_ = NULL;
}

void WaypointFrame::enable()
{
  // activate the frame
  show();
}

void WaypointFrame::disable()
{
  wp_pub_.shutdown();
  hide();
}

void WaypointFrame::saveButtonClicked()
{

  std::cout<< "=======1"<< std::endl;
  QString filename = QFileDialog::getSaveFileName(0,tr("Save Bag"), "waypoints", tr("Bag Files (*.bag)"));
  std::cout<< "=======2"<< std::endl;
  if(filename == "")
    ROS_ERROR("No filename selected");
  else
  {
    std::cout<< "=======3"<< std::endl;
    QFileInfo info(filename);
    std::cout<< "=======4"<< std::endl;
    std::string filn = info.absolutePath().toStdString() + "/" + info.baseName().toStdString() + ".bag";
    ROS_INFO("saving waypoints to %s", filn.c_str());
    
    // rosbag::Bag* bag;
    // try{
    //   bag = new rosbag::Bag();
    //   // bag.open(filn, rosbag::bagmode::Write);
      // rosbag::Bag bag;
      // bag.open("test.bag", rosbag::bagmode::Write);

      // std_msgs::String str;
      // str.data = std::string("foo");

      // // std_msgs::Int32 i;
      // // i.data = 42;

      // bag.write("chatter", ros::Time::now(), str);
      // // bag.write("numbers", ros::Time::now(), i);

      // bag.close();
    // }
    // catch(rosbag::BagIOException e)
    // {
    //   ROS_ERROR("could not open bag %s", filn.c_str());
    //   return;
    // }

    // nav_msgs::Path path;

    // std::map<int, Ogre::SceneNode* >::iterator sn_it;
    // for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    // {
    //   Ogre::Vector3 position;
    //   position = sn_it->second->getPosition();

    //   geometry_msgs::PoseStamped pos;
    //   pos.pose.position.x = position.x;
    //   pos.pose.position.y = position.y;
    //   pos.pose.position.z = position.z;

    //   Ogre::Quaternion quat;
    //   quat = sn_it->second->getOrientation();
    //   pos.pose.orientation.x = quat.x;
    //   pos.pose.orientation.y = quat.y;
    //   pos.pose.orientation.z = quat.z;
    //   pos.pose.orientation.w = quat.w;

    //   path.poses.push_back(pos);
    // }

    // path.header.frame_id = frame_id_.toStdString();

    // bag.write("waypoints", ros::Time::now(), path);
    // bag.close();
  }
}

void WaypointFrame::loadButtonClicked()
{
  QString filename = QFileDialog::getOpenFileName(0,tr("Open Bag"), "~/", tr("Bag Files (*.bag)"));

  if(filename == "")
    ROS_ERROR("No filename selected");
  else
  {
    //Clear existing waypoints
    clearAllWaypoints();

    std::string filn = filename.toStdString();
    ROS_INFO("loading waypoints from %s", filn.c_str());

    rosbag::Bag bag;
    try{
      bag.open(filn, rosbag::bagmode::Read);
    }
    catch(rosbag::BagIOException e)
    {
      ROS_ERROR("could not open bag %s", filn.c_str());
      return;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("waypoints"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
      nav_msgs::Path::ConstPtr p = m.instantiate<nav_msgs::Path>();
      if (p != NULL)
      {
        ROS_INFO("n waypoints %d", (int)p->poses.size());

        for(int i = 0; i < p->poses.size(); i++)
        {
          geometry_msgs::PoseStamped pos = p->poses[i];
          Ogre::Vector3 position;
          position.x = pos.pose.position.x;
          position.y = pos.pose.position.y;
          position.z = pos.pose.position.z;

          Ogre::Quaternion quat;
          quat.x = pos.pose.orientation.x;
          quat.y = pos.pose.orientation.y;
          quat.z = pos.pose.orientation.z;
          quat.w = pos.pose.orientation.w;

          wp_nav_tool_->makeIm(position, quat);
        }
      }
    }
  }
}

void WaypointFrame::publishButtonClicked()
{
  nav_msgs::Path path;

  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
  {
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    path.poses.push_back(pos);
  }

  path.header.frame_id = frame_id_.toStdString();
  wp_pub_.publish(path);
}

void WaypointFrame::clearAllWaypoints()
{
  //destroy the ogre scene nodes
  std::map<int, Ogre::SceneNode* >::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
  {
    scene_manager_->destroySceneNode(sn_it->second);
  }

  //clear the waypoint map and reset index
  sn_map_ptr_->clear();
  *unique_ind_=0;

  //clear the interactive markers
  server_->clear();
  server_->applyChanges();
}

void WaypointFrame::stopExecution()
{
  std_msgs::Empty msg;
  stop_execution_pub_.publish(msg);
  ROS_INFO("Stop execution");
}

void WaypointFrame::continueExecution()
{
  std_msgs::Empty msg;
  continue_execution_pub_.publish(msg);
  ROS_INFO("Continue execution");
}

void WaypointFrame::heightChanged(double h)
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  default_height_ = h;
}

void WaypointFrame::setSelectedMarkerName(std::string name)
{
  selected_marker_name_ = name;
}

void WaypointFrame::poseChanged(double val)
{

  std::map<int, Ogre::SceneNode *>::iterator sn_entry =
      sn_map_ptr_->find(std::stoi(selected_marker_name_.substr(8)));

  if (sn_entry == sn_map_ptr_->end())
    ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else
  {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    visualization_msgs::InteractiveMarker int_marker;
    if(server_->get(wp_name_str, int_marker))
    {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
  }
}

void WaypointFrame::frameChanged()
{

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  QString new_frame = ui_->frame_line_edit->text();

  // Only take action if the frame has changed.
  if((new_frame != frame_id_)  && (new_frame != ""))
  {
    frame_id_ = new_frame;
    ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

    // update the frames for all interactive markers
    std::map<int, Ogre::SceneNode *>::iterator sn_it;
    for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); sn_it++)
    {
      std::stringstream wp_name;
      wp_name << "waypoint" << sn_it->first;
      std::string wp_name_str(wp_name.str());

      visualization_msgs::InteractiveMarker int_marker;
      if(server_->get(wp_name_str, int_marker))
      {
        int_marker.header.frame_id = new_frame.toStdString();
        server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
      }
    }
    server_->applyChanges();
  }
}

void WaypointFrame::topicChanged()
{
  QString new_topic = ui_->topic_line_edit->text();

  // Only take action if the name has changed.
  if(new_topic != output_topic_)
  {
    wp_pub_.shutdown();
    output_topic_ = new_topic;

    if((output_topic_ != "") && (output_topic_ != "/"))
    {
      wp_pub_ = nh_.advertise<nav_msgs::Path>(output_topic_.toStdString(), 1);
    }
  }
}

void WaypointFrame::setWpCount(int size)
{
  std::ostringstream stringStream;
  stringStream << "num wp: " << size;
  std::string st = stringStream.str();

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(st));
}

void WaypointFrame::setConfig(QString topic, QString frame, float height)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->topic_line_edit->blockSignals(true);
  ui_->frame_line_edit->blockSignals(true);
  ui_->wp_height_doubleSpinBox->blockSignals(true);

  ui_->topic_line_edit->setText(topic);
  ui_->frame_line_edit->setText(frame);
  ui_->wp_height_doubleSpinBox->setValue(height);

  ui_->topic_line_edit->blockSignals(false);
  ui_->frame_line_edit->blockSignals(false);
  ui_->wp_height_doubleSpinBox->blockSignals(false);

  }
  topicChanged();
  frameChanged();
  heightChanged(height);
}

void WaypointFrame::getPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  position.x = ui_->x_doubleSpinBox->value();
  position.y = ui_->y_doubleSpinBox->value();
  position.z = ui_->z_doubleSpinBox->value();
  double yaw = ui_->yaw_doubleSpinBox->value();

  tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
  quat.x = qt.x();
  quat.y = qt.y();
  quat.z = qt.z();
  quat.w = qt.w();

  }
}

void WaypointFrame::setPose(Ogre::Vector3& position, Ogre::Quaternion& quat)
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  //block spinbox signals
  ui_->x_doubleSpinBox->blockSignals(true);
  ui_->y_doubleSpinBox->blockSignals(true);
  ui_->z_doubleSpinBox->blockSignals(true);
  ui_->yaw_doubleSpinBox->blockSignals(true);

  ui_->x_doubleSpinBox->setValue(position.x);
  ui_->y_doubleSpinBox->setValue(position.y);
  ui_->z_doubleSpinBox->setValue(position.z);

  tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
  ui_->yaw_doubleSpinBox->setValue(tf::getYaw(qt));

  //enable the signals
  ui_->x_doubleSpinBox->blockSignals(false);
  ui_->y_doubleSpinBox->blockSignals(false);
  ui_->z_doubleSpinBox->blockSignals(false);
  ui_->yaw_doubleSpinBox->blockSignals(false);

  }
}

void WaypointFrame::setWpLabel(Ogre::Vector3 position)
{
  {
  //boost::mutex::scoped_lock lock(frame_updates_mutex_);
  std::ostringstream stringStream;
  stringStream.precision(2);
  stringStream << selected_marker_name_;
  //stringStream << " x: " << position.x << " y: " << position.y << " z: " << position.z;
  std::string label = stringStream.str();

  ui_->sel_wp_label->setText(QString::fromStdString(label));
  }
}

double WaypointFrame::getDefaultHeight()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointFrame::getFrameId()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointFrame::getOutputTopic()
{
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}
} // namespace
