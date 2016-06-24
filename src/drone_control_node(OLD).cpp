/**
\file    drone_control_node.cpp
\brief  This node controls the drone, such that it goes towards the desired pose near the wind-turbine from an initial pose.
 *
 *  A more detailed description of the node. Find doxygen compliant examples.
 *
\author(s)  Olivier Kermorgant, Yuchuan Lang, Syed Yusha Kareem
\date       23/June/2016
*/

//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>

//VISP
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpExponentialMap.h>

//ROS
#include "ros/ros.h"

//ROS msgs

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h> 

//Globals
using namespace std; // for vector
image_transport::Publisher image_pub; 
cv_bridge::CvImagePtr cv_ptr;
ros::Publisher publish_sp;
nav_msgs::Odometry cam_state;

//Subscriber callback to get the camera image from UWSim

bool dataAvailable_image = false ;

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {

//   try
//   {
//     ROS_DEBUG("imageCallback");
//     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    // transfer the ros image to opencv image
//    //cv::Mat srcImage=cv_bridge::toCvCopy(msg, "bgr8")->image;
//    //cv::Mat cv_image=cv_bridge::imgmsg_to_cv2(msg, "bgr8");
//    cv::imshow("mid_view", cv_ptr->image);
//    cv::Mat dstImage,midImage;
//    // do the edge detection and tranform to gray figure
//    cv::Canny( cv_ptr->image, midImage, 50, 200, 3 );
//    cv::cvtColor( midImage, dstImage, cv::COLOR_GRAY2BGR );


//    // the limit length for the segments 
//     int minLineLength = 100;// 50
//     int maxLineGap = 50;    // 10
//    // when using p method to detect the segments, it return x1 y1 x2 and y2 whcih represent the start and end points
//     vector<cv::Vec4i> lines;
//     cv::HoughLinesP( midImage, lines, 1, CV_PI/180, 80,minLineLength,maxLineGap);
//    // draw the segments in the final image
//     for( size_t i = 0; i < lines.size(); i++ )
//     {
//         cv::line( dstImage, cv::Point(lines[i][0], lines[i][1]),cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );

//     }
//     cv::imshow("final_view", dstImage);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//     return;
//   }
// }

//Subscriber callback to get the camera state (pose)

bool dataAvailable_camera_state = false ;

void camera_stateCallback(nav_msgs::Odometry camera_state)
{
    ROS_DEBUG("camera_stateCallback");
    cam_state = camera_state;
    dataAvailable_camera_state = true ;
}

// Main

int main (int argc, char** argv)
{
  //ROS Initialization
  ros::init(argc, argv, "drone_control_node");
  ROS_INFO("drone_control_node connected to roscore");
  ros::NodeHandle nh_("~");//ROS Handler - local namespace.

  //Parameters

  //Subscribing
  
  // ROS_INFO("Subscribing to camera image\n");
  // cv::namedWindow("mid_view");
  // cv::namedWindow("final_view");
  // cv::startWindowThread();
  // image_transport::ImageTransport it(nh_);
  // image_transport::Subscriber sub = it.subscribe("/uwsim/camera", 1, imageCallback);

  ROS_INFO("Subscribing to the camera state (pose)\n");
  ros::Subscriber cam = nh_.subscribe
        <nav_msgs::Odometry>("/camera/state",1,camera_stateCallback); //Careful with cam

  //Publishing 

  publish_sp = nh_.advertise
        <nav_msgs::Odometry>("/camera/state_sp",1);

  //Main loop

    ros::Rate rate(20); //careful here (check if influences speed of convergence)*******
    while (ros::ok()) {

		    ros::spinOnce();

        //Init
        vpColVector v(6);
        //Current homogeneous matrix
        vpTranslationVector t;
        t[0] = cam_state.pose.pose.position.x;
        t[1] = cam_state.pose.pose.position.y;
        t[2] = cam_state.pose.pose.position.z;
        vpQuaternionVector q;
        q[0] = cam_state.pose.pose.orientation.x;
        q[1] = cam_state.pose.pose.orientation.y;
        q[2] = cam_state.pose.pose.orientation.z;
        q[3] = cam_state.pose.pose.orientation.w;
        vpHomogeneousMatrix oMc(t,q);
        vpHomogeneousMatrix cMo = oMc.inverse();
        //Desired homogeneous matrix
        vpTranslationVector t_d;
        t_d[0] = 35.0;
        t_d[1] = 0.0;
        t_d[2] = 20.0;
        vpQuaternionVector q_d;
        q_d[0] = 0.0;
        q_d[1] = -0.707106781187;
        q_d[2] = 0.0;
        q_d[3] = 0.707106781187;
        vpHomogeneousMatrix oMc_d(t_d,q_d);
        vpHomogeneousMatrix cMo_d = oMc_d.inverse();

        // compute a desired velocity
        v = vpExponentialMap::inverse(cMo*cMo_d.inverse());
           
        // compute corresponding pose (update cMo based on current velocity of drone)
        cMo = vpExponentialMap::direct(v,0.01).inverse() * cMo;
   
        vpTime::wait(0.01);

        if( !dataAvailable_camera_state ) {
            ROS_WARN("No camera state available!");
        }else{
        // send corresponding pose to UWSim
        vpTranslationVector te;
        vpQuaternionVector qe;
        cMo.extract(te);   
        cMo.extract(qe);
        nav_msgs::Odometry setpoint;
        setpoint.pose.pose.position.x = te[0];
        setpoint.pose.pose.position.y = te[1];   
        setpoint.pose.pose.position.y = te[2];
        setpoint.pose.pose.orientation.x = qe[0];
        setpoint.pose.pose.orientation.x = qe[1];
        setpoint.pose.pose.orientation.x = qe[2];
        setpoint.pose.pose.orientation.x = qe[3];
        publish_sp.publish(setpoint);
        }
        rate.sleep();
    }
    cv::destroyWindow("mid_view");
    cv::destroyWindow("final_view");
    ROS_INFO("ROS-Node Terminated\n");
}
