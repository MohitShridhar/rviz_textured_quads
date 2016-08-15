#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "single_image_test");
    ros::NodeHandle nh;

    if (argc != 2)
    {
        ROS_ERROR("usage: single_image_test <PATH_TO_TEST_IMAGE>");
        return 0;
    }

    // Load Image
    ROS_INFO("Reading Image ...");
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "odom";
    ros_image.header.stamp = ros::Time::now();
    ROS_INFO("Loaded Image!");

    // Create Mesh
    shape_msgs::Mesh rect;
    
    std::vector<shape_msgs::MeshTriangle> triangles(2);
    triangles.at(0).vertex_indices[0] = 0;
    triangles.at(0).vertex_indices[1] = 1; 
    triangles.at(0).vertex_indices[2] = 2; 
    triangles.at(1).vertex_indices[0] = 1;
    triangles.at(1).vertex_indices[1] = 2; 
    triangles.at(1).vertex_indices[2] = 3; 
    rect.triangles = triangles;

    std::vector<geometry_msgs::Point> vertices(4);
    vertices.at(0).x =  0.0f; vertices.at(1).x =  1.0f; vertices.at(2).x =  0.0f; vertices.at(3).x =  1.0f;
    vertices.at(0).y = -1.0f; vertices.at(1).y = -1.0f; vertices.at(2).y =  0.0f; vertices.at(3).y =  0.0f;
    vertices.at(0).z =  0.0f; vertices.at(1).z =  0.0f; vertices.at(2).z =  0.0f; vertices.at(3).z =  0.0f;
    rect.vertices = vertices;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/mesh_image", 1);
    ros::Publisher shape_pub = nh.advertise<shape_msgs::Mesh>("/mesh_shape", 1);
    ros::Rate loop_rate(1);


    while (nh.ok()) 
    {
      // publish image
      image_pub.publish(ros_image);
      
      // publish mesh
      shape_pub.publish(rect);

      loop_rate.sleep();

    }

    ros::shutdown();

    return 0;
}