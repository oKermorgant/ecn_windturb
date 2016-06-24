//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>
//ROS
#include "ros/ros.h"
//openCV
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
string TEXT="Yuchuan" ;
cv::Mat image;
image_transport::Publisher  pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
   // transfer the ros image to opencv image
   cv::Mat srcImage=cv_bridge::toCvCopy(msg, "bgr8")->image;
   //cv::Mat cv_image=cv_bridge::imgmsg_to_cv2(msg, "bgr8");
   cv::imshow("Image_Window", srcimage);
   cv::Mat dstImage,midImage;
   // do the edge detection and tranform to gray figure
   cv::Canny( srcImage, midImage, 50, 200, 3 );
   cv::cvtColor( midImage, dstImage, cv::COLOR_GRAY2BGR );


   // the limit length for the segments 
    minLineLength = 100;// 50
    maxLineGap = 50;    // 10
   // when using p method to detect the segments, it return x1 y1 x2 and y2 whcih represent the start and end points
    vector<cv::Vec4i> lines;
    lines=cv::HoughLinesP( midImage, lines, 1, CV_PI/180, 80,minLineLength,maxLineGap);
   // draw the segments in the final image
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::line( dstImage, cv::Point(lines[i][0], lines[i][1]),cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
        //(1,cv_AA);
       bool m=1;
    
                      if(m)
                    {   std::vector<double>  x1,y1,x2,y2;
                      x1[i]=lines[0];
                      y1[i]=lines[1];
                      x2[i]=lines[2];
                      y2[i]=lines[3];
                     // cv::line(detImage,(x1,y1),(x2,y2),(0,255,0),2);
                      rho = sqrt((x1*y2 - x2*y1)*(x1*y2 - x2*y1)/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2))/100;
                      theta = arctan2((x1 - x2)/(float(x1)*y2 - x2*y1),(-y1 + y2)/(float(x1)*y2 - x2*y1));
                      std::vector<double> rt=([rho,theta]);
                      //vector centerP[i+1]

                        // cluster
                        //rt = pl.float32(pl.array(rt))
                        //K = 4
                        //criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1)
                        //ret,label,center = cv2.kmeans(rt,K,criteria,10,cv2.KMEANS_RANDOM_CENTERS, 1)
                        //pl.close('all')
                        //cols = [ 'b', 'g', 'r', 'c', 'm']
                        //for i in xrange(K):
                        //  here = rt[label.ravel()==i]
                        //pl.plot(here[:,0],here[:,1], '.'+cols[i])
                        //pl.plot(center[i,0],center[i,1],'D'+cols[i])
                        //

                          double a = cos(theta), b = sin(theta);
                          double x0 = a*rho*100, y0 = b*rho*100;
                          cv::Point pt1(cv::cvRound(x0 + 1000*(-b)),
                          cv::cvRound(y0 + 1000*(a)));
                          cv::Point pt2(cv::cvRound(x0 - 1000*(-b)),
                          cv::cvRound(y0 - 1000*(a)));
                          cv::line( dstImage, pt1, pt2, Scalar(0,0,255), 3, 8 );
                     }
       }
       for( i = 0; i < lines.size()-1; i++ )
       { 
        std::vector<double> Px,Py;
        Px[i]=((x1[i]*y2[i]-y1[i]*x2[i])*(x1[i+1]-x2[i+1])-(x1[i]-x2[i])*(x1[i+1]*y2[i+1]-y1[i+1]*x2[i+1]))/((x1[i]-x2[i])*(y1[i+1]-y2[i+1])-(y1[i]-y2[i])*(x1[i+1]-x2[i+1]));
        Py[i]=((x1[i]*y2[i]-y1[i]*x2[i])*(y1[i+1]-y2[i+1])-(y1[i]-y2[i])*(x1[i+1]*y2[i+1]-y1[i+1]*x2[i+1]))/((x1[i]-x2[i])*(y1[i+1]-y2[i+1])-(y1[i]-y2[i])*(x1[i+1]-x2[i+1]));
       }
       double Pxc,Pyc;
       Pxc=Pyc=0;
       for( i = 0; i < lines.size()-1; i++ )
       {
        Pxc=Pxc+Px[i];
        Pyc=Pyc+Py[i];
       }
      Pxc=Pxc/(lines.size()-1);
      Pyc=Pyc/(lines.size()-1);

          minLineLength = 100;
          maxLineGap = 50;
          lines = cv2.HoughLinesP(thr,1,pl.pi/180,50,minLineLength,maxLineGap);

          // control law
          if(useDiffAngles)
            {
                for(i=0;i<2;++i)
                {
                    i2 = ((i+1)%3) +1;
                    dt[i] = l[i2].error(l[i+1])[1];
                    e[m] = dt[i] - dtd[i];
                    Ldt = l[i2].interaction() - l[i+1].interaction();
                    for(unsigned int j=0;j<6;++j)
                    {
                        L[m][j] = Ldt[1][j];
                    }

                    m += 1;
                }
            }




   cv::putText(cv_image,TEXT,cv::Point2f(100,100),CV_FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,255,255),2,CV_AA);
   cv::waitKey(30);
   sensor_msgs::ImagePtr rosmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",cv_image).toImageMsg();
   //sensor_msgs::ImagePtr rosmsg = cv_bridge::cv2_to_imgmsg(cv_image, "bgr8");
  //cv_bridge::CvImage(std_msgs::Header(), "bgr8",image).toImageMsg();
   //ros::Rate loop_rate(5);
   pub.publish(rosmsg);
   //loop_rate.sleep();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "baxter_img_node");
    ros::NodeHandle nh;
    ROS_INFO("Connected !!!!!");

    image_transport::ImageTransport it(nh);
    cv::namedWindow("Image_Window");
    cv::startWindowThread();
    //Publisher
     pub= it.advertise("/image_out", 1);

    //Subscribing
    ROS_INFO("Subscribing to topics\n");
    image_transport::Subscriber sub = it.subscribe("/image_in", 1, imageCallback);

    ros::Rate rate(20) ;

    ROS_INFO("Image sampling period");
    while (ros::ok()){

     // Attend callbacks
     ros::spinOnce();
     rate.sleep();


    } 
     cv::destroyWindow("Image_window");
    ROS_INFO("ROS-Node Terminated\n");
}
