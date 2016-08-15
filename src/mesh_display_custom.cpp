/*
 * MeshDisplayCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on the rviz image display class.
 *
 * Latest changes (12/11/2012):
 * - fixed segfault issues
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h> 
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreFrustum.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/render_panel.h"
#include "rviz/validate_floats.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include <image_transport/camera_common.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mesh_display_custom.h"

namespace rviz
{

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
    bool valid = true;
    valid = valid && validateFloats( msg.D );
    valid = valid && validateFloats( msg.K );
    valid = valid && validateFloats( msg.R );
    valid = valid && validateFloats( msg.P );
    return valid;
}

MeshDisplayCustom::MeshDisplayCustom()
    : ImageDisplayBase()
    , time_since_last_transform_( 0.0f )
    , mesh_node_(NULL)
    , projector_node_(NULL)
    , decal_frustum_(NULL)
    , manual_object_(NULL)
    , initialized_(false)
{
    image_alpha_property_ = new FloatProperty( "Image Alpha", 1.0f,
                                               "Amount of transparency for the mesh with image texture overlay.", this, SLOT( updateMeshProperties() ) );

    mesh_topic_property_ = new RosTopicProperty( "Mesh Topic", "",
                                            QString::fromStdString( ros::message_traits::datatype<shape_msgs::Mesh>() ),
                                            "shape_msgs::Mesh topic to subscribe to.",
                                            this, SLOT( updateTopic() ));

    mesh_alpha_property_ = new FloatProperty( "Mesh Alpha", 0.6f,
                                              "Amount of transparency for the mesh.", this, SLOT( updateMeshProperties() ) );

    mesh_color_property_ = new ColorProperty( "Mesh Color", QColor( 255, 255, 255 ),
                                              "Color to mesh when not overlayed by image texture.", this, SLOT( updateMeshProperties() ) );

    // this shouldn't necessarily be here, and we should get this from a camera topic with camera info
    position_property_ = new VectorProperty( "Projector Position", Ogre::Vector3::ZERO,
                                             "position of the texture projector object in /world",
                                             this, SLOT( updateMeshProperties() ) );
    rotation_property_ = new QuaternionProperty("Projector Rotation", Ogre::Quaternion::IDENTITY,"rotation of the texture projector object",this,SLOT(updateMeshProperties()));

}

MeshDisplayCustom::~MeshDisplayCustom()
{
    unsubscribe();
    caminfo_tf_filter_->clear();
    delete caminfo_tf_filter_;
}

void MeshDisplayCustom::onInitialize()
{
    ImageDisplayBase::onInitialize();

    // it will only accept images if the frame in camera info is resolved
    caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                                         queue_size_property_->getInt(), update_nh_ );

    context_->getSceneManager()->addRenderQueueListener(this);

    caminfo_tf_filter_->connectInput(caminfo_sub_);
    caminfo_tf_filter_->registerCallback(boost::bind(&MeshDisplayCustom::caminfoCallback, this, _1));
}

void MeshDisplayCustom::createProjector()
{
    decal_frustum_ = new Ogre::Frustum();

    projector_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    projector_node_->attachObject(decal_frustum_);

    Ogre::SceneNode* filter_node;

    //back filter
    filter_frustum_.push_back(new Ogre::Frustum());
    filter_frustum_.back()->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    filter_node = projector_node_->createChildSceneNode();
    filter_node->attachObject(filter_frustum_.back());
    filter_node->setOrientation(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_Y));
}

void MeshDisplayCustom::addDecalToMaterial(const Ogre::String& matName)
{
    Ogre::MaterialPtr mat = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(matName);
    mat->setCullingMode(Ogre::CULL_NONE);
    Ogre::Pass* pass = mat->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    //pass->setLightingEnabled(true);

    // need the decal_filter to avoid back projection
    Ogre::String resource_group_name = "decal_textures_folder";
    Ogre::ResourceGroupManager& resource_manager = Ogre::ResourceGroupManager::getSingleton();
    if(!resource_manager.resourceGroupExists(resource_group_name))
    {
        resource_manager.createResourceGroup(resource_group_name);
        resource_manager.addResourceLocation(ros::package::getPath("rviz_plugin_image_mesh")+"/tests/textures/", "FileSystem", resource_group_name, false);
        resource_manager.initialiseResourceGroup(resource_group_name);
    }
    // loads files into our resource manager
    resource_manager.loadResourceGroup(resource_group_name);

    Ogre::TextureUnitState* tex_state = pass->createTextureUnitState();//"Decal.png");
    tex_state->setTextureName(texture_.getTexture()->getName());
    tex_state->setProjectiveTexturing(true, decal_frustum_);

    tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    tex_state->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);
    tex_state->setColourOperation(Ogre::LBO_REPLACE); //don't accept additional effects

    for(int i = 0; i < filter_frustum_.size(); i++)
    {
        tex_state = pass->createTextureUnitState("Decal_filter.png");
        tex_state->setProjectiveTexturing(true, filter_frustum_[i]);
        tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
        tex_state->setTextureFiltering(Ogre::TFO_NONE);
    }
}

void MeshDisplayCustom::setPose()
{
    if(projector_node_ == NULL)
        return;

    projector_node_->setPosition(position_property_->getVector().x,
                            position_property_->getVector().y,
                            position_property_->getVector().z);
    projector_node_->setOrientation(rotation_property_->getQuaternion().w,
                               rotation_property_->getQuaternion().x,
                               rotation_property_->getQuaternion().y,
                               rotation_property_->getQuaternion().z);
}

void MeshDisplayCustom::updateMesh( const shape_msgs::Mesh::ConstPtr& mesh )
{
    boost::mutex::scoped_lock lock( mesh_mutex_ );

    // create our scenenode and material
    load();

    // set properties
    setPose();

    if (!manual_object_)
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "MeshObject" << count++;
        manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
        mesh_node_->attachObject(manual_object_);
    }

    // If we have the same number of tris as previously, just update the object
    if (last_mesh_.vertices.size() > 0 && mesh->vertices.size()*2 == last_mesh_.vertices.size())
    {
        manual_object_->beginUpdate(0);
    }
    else // Otherwise clear it and begin anew
    {
        manual_object_->clear();
        manual_object_->estimateVertexCount(mesh->vertices.size()*2);
        manual_object_->begin(mesh_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    }

    const std::vector<geometry_msgs::Point>& points = mesh->vertices;
    for(size_t i = 0; i < mesh->triangles.size(); i++)
    {
        // make sure we have front-face/back-face triangles
        for(int side = 0; side < 2; side++)
        {
            std::vector<Ogre::Vector3> corners(3);
            for(size_t c = 0; c < 3; c++)
            {
                size_t corner = side ? 2-c : c; // order of corners if side == 1
                corners[corner] = Ogre::Vector3(points[mesh->triangles[i].vertex_indices[corner]].x, points[mesh->triangles[i].vertex_indices[corner]].y, points[mesh->triangles[i].vertex_indices[corner]].z);
            }
            Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
            normal.normalise();

            for(size_t c = 0; c < 3; c++)
            {
                manual_object_->position(corners[c]);
                manual_object_->normal(normal);
            }
        }
    }

    manual_object_->end();

    mesh_material_->setCullingMode(Ogre::CULL_NONE);

    last_mesh_ = *mesh;
}

void MeshDisplayCustom::updateMeshProperties()
{
    // update transformations
    setPose();

    // update color/alpha
    Ogre::Technique* technique = mesh_material_->getTechnique(0);
    Ogre::Pass* pass = technique->getPass(0);

    Ogre::ColourValue self_illumination_color(0.0f, 0.0f, 0.0f, mesh_alpha_property_->getFloat());
    pass->setSelfIllumination(self_illumination_color);

    Ogre::ColourValue diffuse_color(mesh_color_property_->getColor().redF(), mesh_color_property_->getColor().greenF(), mesh_color_property_->getColor().blueF(), mesh_alpha_property_->getFloat());
    pass->setDiffuse(diffuse_color);

    Ogre::ColourValue ambient_color(mesh_color_property_->getColor().redF()/2.0f, mesh_color_property_->getColor().greenF()/2.0f, mesh_color_property_->getColor().blueF()/2.0f, mesh_alpha_property_->getFloat());
    pass->setAmbient(ambient_color);

    Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
    pass->setSpecular(specular_color);

    Ogre::Real shininess = 64.0f;
    pass->setShininess(shininess);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);

    context_->queueRender();
}

void MeshDisplayCustom::updateTopic()
{
    unsubscribe();
    subscribe();
}

void MeshDisplayCustom::subscribe()
{
    if ( !isEnabled() )
    {
        return;
    }

    if( !mesh_topic_property_->getTopic().isEmpty() )
    {
        try
        {
            pose_sub_ = nh_.subscribe( mesh_topic_property_->getTopicStd(), 1, &MeshDisplayCustom::updateMesh, this );
            setStatus( StatusProperty::Ok, "Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
        }
    }

    if( !topic_property_->getTopic().isEmpty() )
    {
        std::string target_frame = fixed_frame_.toStdString();
        ImageDisplayBase::enableTFFilter(target_frame);

        ImageDisplayBase::subscribe();

        std::string topic = topic_property_->getTopicStd();
        std::string caminfo_topic = image_transport::getCameraInfoTopic(topic);

        try
        {
          caminfo_sub_.subscribe( update_nh_, caminfo_topic, 1 );
          setStatus( StatusProperty::Ok, "Camera Info", "OK" );
        }
        catch( ros::Exception& e )
        {
          setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
        }
    }
}

void MeshDisplayCustom::unsubscribe()
{
    ImageDisplayBase::unsubscribe();
    caminfo_sub_.unsubscribe();
    pose_sub_.shutdown();
}

void MeshDisplayCustom::load()
{
    if(mesh_node_ != NULL)
        return;

    static int count = 0;
    std::stringstream ss;
    ss << "MeshNode" << count++ << "Group";
    Ogre::MaterialManager& material_manager = Ogre::MaterialManager::getSingleton();
    Ogre::String resource_group_name =  ss.str();

    Ogre::ResourceGroupManager& rg_mgr = Ogre::ResourceGroupManager::getSingleton();

    Ogre::String material_name = resource_group_name+"MeshMaterial";

    if(!rg_mgr.resourceGroupExists(resource_group_name))
    {
        rg_mgr.createResourceGroup(resource_group_name);

        mesh_material_ = material_manager.create(material_name,resource_group_name);
        Ogre::Technique* technique = mesh_material_->getTechnique(0);
        Ogre::Pass* pass = technique->getPass(0);

        Ogre::ColourValue self_illumnation_color(0.0f, 0.0f, 0.0f, mesh_alpha_property_->getFloat());
        pass->setSelfIllumination(self_illumnation_color);

        Ogre::ColourValue diffuse_color(mesh_color_property_->getColor().redF(), mesh_color_property_->getColor().greenF(), mesh_color_property_->getColor().blueF(), mesh_alpha_property_->getFloat());
        pass->setDiffuse(diffuse_color);

        Ogre::ColourValue ambient_color(mesh_color_property_->getColor().redF()/2.0f, mesh_color_property_->getColor().greenF()/2.0f, mesh_color_property_->getColor().blueF()/2.0f, mesh_alpha_property_->getFloat());
        pass->setAmbient(ambient_color);

        Ogre::ColourValue specular_color(1.0f, 1.0f, 1.0f, 1.0f);
        pass->setSpecular(specular_color);

        Ogre::Real shininess = 64.0f;
        pass->setShininess(shininess);

        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        mesh_material_->setCullingMode(Ogre::CULL_NONE);
    }

    mesh_node_ = this->scene_node_->createChildSceneNode();
}

void MeshDisplayCustom::onEnable()
{
    subscribe();
}

void MeshDisplayCustom::onDisable()
{
    unsubscribe();
}

void MeshDisplayCustom::update( float wall_dt, float ros_dt )
{
    time_since_last_transform_ += wall_dt;

//    just added automatic rotation to make it easier  to test things
   // if(projector_node_ != NULL)
   // {
   //     projector_node_->rotate(Ogre::Vector3::UNIT_Y, Ogre::Degree(wall_dt * 50));
   //     rotation_property_->setQuaternion(projector_node_->getOrientation());
   // }

    if( !topic_property_->getTopic().isEmpty() )
    {
        std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());
        if(caminfo_sub_.getTopic().compare(caminfo_topic) != 0)
        {
            //std::cout<<"updating topic" <<std::endl;

            caminfo_sub_.unsubscribe();
            try
            {
                caminfo_sub_.subscribe( update_nh_, caminfo_topic, 1 );
                // std::cout<<"The subscription happens"<<std::endl;
                setStatus( StatusProperty::Ok, "Camera Info", "OK" );
            }
            catch( ros::Exception& e )
            {
                setStatus( StatusProperty::Error, "Camera Info", QString( "Error subscribing: ") + e.what() );
            }
        }

        try
        {
            updateCamera(texture_.update());
        }
        catch( UnsupportedImageEncoding& e )
        {
            setStatus(StatusProperty::Error, "Image", e.what());
        }
    }
}

bool MeshDisplayCustom::updateCamera(bool update_image)
{
    if(update_image)
    {
        boost::mutex::scoped_lock lock( caminfo_mutex_ );

        last_info_ = current_caminfo_;
        last_image_ = texture_.getImage();
    }
    if(!last_info_ || !last_image_)
    {        
        return false;
    }

    if(!validateFloats( *last_info_ ))
    {
        setStatus( StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)" );
        return false;
    }

    boost::mutex::scoped_lock lock( mesh_mutex_ );

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    context_->getFrameManager()->getTransform( last_image_->header.frame_id, last_image_->header.stamp, position, orientation );

    position = Ogre::Vector3(0.5f, -0.5f, 1.0f/590.0f);
    orientation = Ogre::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);


    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_Z );

    // std::cout << "CameraInfo dimensions: " << last_info_->width << " x " << last_info_->height << std::endl;
    // std::cout << "Texture dimensions: " << last_image_->width << " x " << last_image_->height << std::endl;
    //std::cout << "Original image dimensions: " << last_image_->width*full_image_binning_ << " x " << last_image_->height*full_image_binning_ << std::endl;


    float img_width  = last_info_->width;//image->width*full_image_binning_;
    float img_height = last_info_->height;//image->height*full_image_binning_;

    // If the image width/height is 0 due to a malformed caminfo, try to grab the width from the image.
    if( img_width <= 0 )
    {
      ROS_ERROR( "Malformed CameraInfo on camera [%s], width = 0", qPrintable( getName() ));
      // use texture size, but have to remove border from the perspective calculations
      img_width = texture_.getWidth()-2;
    }

    if (img_height <= 0)
    {
        ROS_ERROR( "Malformed CameraInfo on camera [%s], height = 0", qPrintable( getName() ));
        // use texture size, but have to remove border from the perspective calculations
        img_height = texture_.getHeight()-2;
    }

    // if even the texture has 0 size, return
    if( img_height <= 0.0 || img_width <= 0.0 )
    {
        setStatus( StatusProperty::Error, "Camera Info",
                 "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0) and texture." );
        return false;
    }

    // calculate projection matrix
    if(last_info_->P[0] != 0)
    {
        double fx = last_info_->P[0];
        double fy = last_info_->P[5];

        // Add the camera's translation relative to the left camera (from P[3]);
        double tx = -1 * (last_info_->P[3] / fx);
        Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
        position = position + (right * tx);

        double ty = -1 * (last_info_->P[7] / fy);
        Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
        position = position + (down * ty);

        if( !validateFloats( position ))
        {
            ROS_ERROR( "position error");
            setStatus( StatusProperty::Error, "Camera Info", "CameraInfo/P resulted in an invalid position calculation (nans or infs)" );
            return false;
        }

        if(projector_node_ != NULL)
        {
            projector_node_->setPosition( position );
            projector_node_->setOrientation( orientation );

            // std::cout << position << std::endl;
            // std::cout << orientation << std::endl;
        }

        // calculate the projection matrix
        double cx = last_info_->P[2];
        double cy = last_info_->P[6];

        double far_plane = 100;
        double near_plane = 0.01;

        Ogre::Matrix4 proj_matrix;
        proj_matrix = Ogre::Matrix4::ZERO;

        proj_matrix[0][0]= 2.0 * fx/img_width;
        proj_matrix[1][1]= 2.0 * fy/img_height;

        proj_matrix[0][2]= 2.0 * (0.5 - cx/img_width);
        proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5);

        proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
        proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

        proj_matrix[3][2]= -1;

        hfov_ = atan( 1.0f / proj_matrix[0][0] ) * 2.0f * 57.2957795f;
        vfov_ = atan( 1.0f / proj_matrix[1][1] ) * 2.0f * 57.2957795f;

        if(decal_frustum_ != NULL)
            decal_frustum_->setCustomProjectionMatrix(true, proj_matrix);

        // ROS_INFO(" Camera (%f, %f)", proj_matrix[0][0], proj_matrix[1][1]);
        // ROS_INFO(" Render Panel: %x   Viewport: %x", render_panel_, render_panel_->getViewport());
    }

    setStatus( StatusProperty::Ok, "Time", "ok" );
    setStatus( StatusProperty::Ok, "Camera Info", "ok" );

    if(mesh_node_ != NULL && filter_frustum_.size() == 0)
    {
        createProjector();

        addDecalToMaterial(mesh_material_->getName());
    }

    return true;
}

void MeshDisplayCustom::clear()
{
    texture_.clear();
    context_->queueRender();

    current_caminfo_.reset();
    setStatus( StatusProperty::Warn, "Camera Info",
               "No CameraInfo received on [" + QString::fromStdString( caminfo_sub_.getTopic() ) + "].  Topic may not exist.");
    setStatus( StatusProperty::Warn, "Image", "No Image received");
}

void MeshDisplayCustom::reset()
{
    ImageDisplayBase::reset();
    clear();
}

/* This is called by incomingMessage(). */
void MeshDisplayCustom::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
    //std::cout<<"camera image received"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;

    // simply converting every image to RGBA
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("MeshDisplayCustom: cv_bridge exception: %s", e.what());
        return;
    }

    // update image alpha
    for(int i = 0; i < cv_ptr->image.rows; i++)
    {
        for(int j = 0; j < cv_ptr->image.cols; j++)
        {
            cv::Vec4b& pixel = cv_ptr->image.at<cv::Vec4b>(i,j);
            pixel[3] = image_alpha_property_->getFloat()*255;
        }
    }

    // add completely white transparent border to the image so that it won't replicate colored pixels all over the mesh
    cv::Scalar value(255,255,255,0);
    cv::copyMakeBorder(cv_ptr->image,cv_ptr->image,1,1,1,1,cv::BORDER_CONSTANT,value);

    // Output modified video stream
    texture_.addMessage(cv_ptr->toImageMsg());
}

void MeshDisplayCustom::caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg )
{
    //std::cout<<"camera info received"<<std::endl;
    boost::mutex::scoped_lock lock( caminfo_mutex_ );
    current_caminfo_ = msg;
}


void MeshDisplayCustom::updateQueueSize()
{
    caminfo_tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
    ImageDisplayBase::updateQueueSize();
}


void MeshDisplayCustom::fixedFrameChanged()
{
    std::string targetFrame = fixed_frame_.toStdString();
    caminfo_tf_filter_->setTargetFrame(targetFrame);

    ImageDisplayBase::fixedFrameChanged();
}

} // namespace rviz
