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

#ifndef KR_RVIZ_PLUGIN_WAYPOINT_FRAME_
#define KR_RVIZ_PLUGIN_WAYPOINT_FRAME_

#ifndef Q_MOC_RUN
#include <boost/thread/mutex.hpp>
#endif

#include <QWidget>
#include <std_msgs/Empty.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>

#include "ui_WaypointNavigation.h"

namespace Ogre
{
class SceneNode;
class Vector3;
class SceneManager;
class Quaternion;
}

namespace rviz
{
class DisplayContext;
}

namespace interactive_markers
{
class InteractiveMarkerServer;
}

namespace Ui
{
class WaypointNavigationWidget;
}

namespace waypoint_nav_plugin
{
class WaypointNavTool;
}

namespace waypoint_nav_plugin
{

class WaypointFrame : public QWidget
{
  friend class WaypointNavTool;
  Q_OBJECT

public:
  WaypointFrame(rviz::DisplayContext *context, std::map<int, Ogre::SceneNode* >* map_ptr, interactive_markers::InteractiveMarkerServer* server, int* unique_ind, QWidget *parent = 0, WaypointNavTool* wp_tool=0);
  ~WaypointFrame();

  void enable();
  void disable();

  void setWpCount(int size);
  void setConfig(QString topic, QString frame, float height);
  void setWpLabel(Ogre::Vector3 position);
  void setSelectedMarkerName(std::string name);
  void setPose(Ogre::Vector3& position, Ogre::Quaternion& quat);


  double getDefaultHeight();
  QString getFrameId();
  QString getOutputTopic();
  void getPose(Ogre::Vector3& position, Ogre::Quaternion& quat);

protected:

  Ui::WaypointNavigationWidget *ui_;
  rviz::DisplayContext* context_;

private Q_SLOTS:
  void publishButtonClicked();
  void clearAllWaypoints();
  void heightChanged(double h);
  void frameChanged();
  void topicChanged();
  void poseChanged(double val);
  void saveButtonClicked();
  void loadButtonClicked();
  void stopExecution();
  void continueExecution();

private:

  ros::NodeHandle nh_;
  ros::Publisher wp_pub_;
  ros::Publisher stop_execution_pub_;
  ros::Publisher continue_execution_pub_;

  WaypointNavTool* wp_nav_tool_;
  //pointers passed via contructor
  std::map<int, Ogre::SceneNode* >* sn_map_ptr_;
  Ogre::SceneManager* scene_manager_;
  int* unique_ind_;

  interactive_markers::InteractiveMarkerServer* server_;

  //default height the waypoint must be placed at
  double default_height_;

  // The current name of the output topic.
  QString output_topic_;
  QString frame_id_;

  //mutex for changes of variables
  boost::mutex frame_updates_mutex_;

  std::string selected_marker_name_;

};

}

#endif
