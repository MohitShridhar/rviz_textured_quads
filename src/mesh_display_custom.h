/*
 * MeshDisplayCustom declaration.
 *
 * Author: Felipe Bacim.
 *
 * help with selection of robot parts 
 */
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_MESH_DISPLAY_H
#define RVIZ_MESH_DISPLAY_H

#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreRenderQueueListener.h>

#include <rviz_plugin_image_mesh/RvizDisplayImages.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
class ManualObject;
}

namespace rviz
{
class Axes;
class RenderPanel;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
class VectorProperty;
class StringProperty;
class QuaternionProperty;
}

namespace rviz
{

/**
 * \class MeshDisplayCustom
 * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
 */
class MeshDisplayCustom: public rviz::ImageDisplayBase,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
{
Q_OBJECT
public:
  MeshDisplayCustom();
  virtual ~MeshDisplayCustom();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();
  virtual void fixedFrameChanged();

private Q_SLOTS:
  void updateMeshProperties();
  void updateTopic();
  void updateDisplayImages();
  void updateName();
  virtual void updateQueueSize();

protected:
  void setPose();
  virtual void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  // This is called by incomingMessage().
  virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg);
  void processImage(const sensor_msgs::Image& msg);

  virtual void subscribe();
  virtual void unsubscribe();

private:
  void clear();
  void updateStatus();
  bool updateCamera(bool update_image);
  void caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg );

  void createProjector();
  void addDecalToMaterial(const Ogre::String& matName);
  void updateMesh( const shape_msgs::Mesh::ConstPtr& mesh );
  void updateImageMeshes( const rviz_plugin_image_mesh::RvizDisplayImages::ConstPtr& images );

  float time_since_last_transform_;

  RosTopicProperty* mesh_topic_property_;
  RosTopicProperty* display_images_topic_property_;
  FloatProperty* mesh_alpha_property_;
  FloatProperty* image_alpha_property_;
  ColorProperty* mesh_color_property_;
  VectorProperty* position_property_;
  StringProperty* type_property_;
  QuaternionProperty* rotation_property_;

  geometry_msgs::Pose pose_;
  shape_msgs::Mesh last_mesh_;

  ros::NodeHandle nh_;

  //This deals with the camera info
  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  tf::MessageFilter<sensor_msgs::CameraInfo>* caminfo_tf_filter_;

  sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
  boost::mutex caminfo_mutex_;

  // hold the last information received
  sensor_msgs::CameraInfo::ConstPtr last_info_;
  sensor_msgs::Image::ConstPtr last_image_;
  float hfov_, vfov_;

  Ogre::SceneNode* mesh_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr mesh_material_;
  ROSImageTexture texture_;

  ros::Subscriber pose_sub_;
  ros::Subscriber rviz_display_images_sub_;

  Ogre::Frustum* decal_frustum_;
  std::vector<Ogre::Frustum*> filter_frustum_; //need multiple filters (back, up, down, left, right)
  Ogre::SceneNode* projector_node_;

  std::vector<RenderPanel*> render_panel_list_;
  RenderPanel* render_panel_; // this is the active render panel

  bool initialized_;

  boost::mutex mesh_mutex_;
};

} // namespace rviz

#endif


