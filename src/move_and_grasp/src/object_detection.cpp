/*
    Date: 2019/03/13
Author: Xu Yucheng 
    Abstract: object detection msg receive, 3D-position predict with PCL
*/
#define PI 3.1415926

// common headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <sstream>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>

// user-defined ROS message type
#include <move_and_grasp/ObjectPosition.h>

using namespace std;
using namespace pcl;
using namespace cv;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_astra (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];
    
bool find_object = false;
bool pub_obj_position = false;
int camera_width = 640;
int camera_height = 480;
int object_x = 0;
int object_y = 0;
int origin_index = 0;
int valid_index = 0;
cv::Mat cv_image;

int mouse_x = 0;
int mouse_y = 0;
int mouse_index = 0;

void on_mouse (int event, int x, int y, int flags, void* param)
{
    Mat *im = reinterpret_cast<Mat*>(param);

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            //left button down respond, save the object position in rgb graph
            mouse_y = y;
            mouse_x = x;
            mouse_index = y*camera_width + x;
            int new_idx = 0;
            std::cout << "at(" << x << "," << y << ")" <<std::endl;
            object_x = x;
            object_y = y;
            std::cout << "3D position(" << cloud_astra->points[mouse_index].x <<","
                                        << cloud_astra->points[mouse_index].y << "," 
                                        << cloud_astra->points[mouse_index].z << ")" <<std::endl;     
            find_object = true;
            break;
    }
   
}

class object_detection
{
private:
    // ROS parameters
    std::string sub_pcl_topic_name;
    std::string sub_image_raw_topic_name;
    std::string pub_object_pos_topic_name;

    // ROS subscriber & publisher
    ros::Subscriber pcl_sub;
    ros::Subscriber img_sub;
    ros::Subscriber control_sub;
    ros::Publisher obj_pub;

   
    
    //if pcl not a number in the position detected, find the nearest valid pcl
    int find_near_valid(int idx)
    {   
        double temp_min = 9999999999;
        int return_idx = idx;
        int obj_row = idx/camera_width;
        int obj_col = idx%camera_width;

        for (int row=0; row<camera_height; row++)
        {
            for (int col=0; col<camera_width; col++)
            {
                if (!isnan(cloud_astra->points[row*camera_width+col].x) &&
                    !isnan(cloud_astra->points[row*camera_width+col].y) &&
                    !isnan(cloud_astra->points[row*camera_width+col].z))
                {
                    double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
                    if (dis < temp_min)
                    {
                        return_idx = row*camera_width + col;
                        temp_min = dis;
                    }
                }
            }
        }
        valid_index = return_idx;
        return return_idx;
    }

    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        cv::imshow("find_object_window",img);
        cv::waitKey(10);
        if (find_object && !cloud_astra->empty())
        {
            int ori_row = origin_index / camera_width;
            int ori_col = origin_index % camera_width;
            int val_row = valid_index / camera_width;
            int val_col = valid_index % camera_width;
            
            Point ori(ori_col, ori_row);
            Point val(val_col, val_row);

            circle(img, ori, 3, Scalar(0,0,255), -1);
            circle(img, val, 3, Scalar(255,0,0), -1);

            cv::imshow("object_detection", img);
            cv::waitKey(10);
            get_object_position();
        }
    }

    void pclCallback(sensor_msgs::PointCloud2 msg)
    {
        // PointCloud2 is ROS message type, cloud_astra is pcl message type
        // We use this function to tranform msg type and save the point cloud
        pcl::fromROSMsg (msg, *cloud_astra);
        
    }

    // Get the 3D object position described in base_link and publish
    void get_object_position()
    {
        if (cloud_astra->empty())
        {
            ROS_INFO ("Waiting for pcl transform");
            sleep (2);
        }
        else
        {
            geometry_msgs::PointStamped cam_pos;
            geometry_msgs::PointStamped base_point;
            cam_pos.header.frame_id = "/astra_depth_optical_frame";
            origin_index = object_y * camera_width + object_x;
            move_and_grasp::ObjectPosition pos;
            tf::TransformListener pListener;

            if (isnan(cloud_astra->points[origin_index].x) || 
                isnan(cloud_astra->points[origin_index].y) ||
                isnan(cloud_astra->points[origin_index].z))
            {  
                int new_index = find_near_valid(origin_index);
                cam_pos.header.stamp = ros::Time(0);
                cam_pos.point.x = cloud_astra->points[new_index].x;
                cam_pos.point.y = cloud_astra->points[new_index].y;
                cam_pos.point.z = cloud_astra->points[new_index].z;
				
                try
                {
                    pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
                    pListener.transformPoint("/base_link", cam_pos, base_point);
                    ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                            cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
                }
                
            }
            else
            {
                cam_pos.header.stamp = ros::Time(0);
                cam_pos.point.x = cloud_astra->points[origin_index].x;
                cam_pos.point.y = cloud_astra->points[origin_index].y;
                cam_pos.point.z = cloud_astra->points[origin_index].z;
                try
                {
                    pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
                    pListener.transformPoint("/base_link", cam_pos, base_point);
                    ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                            cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
                }
            }
            if (!pub_obj_position)
            {
                // publish only once
                pos.header.frame_id = "/base_link";
                pos.x = base_point.point.x;
                pos.y = base_point.point.y;
                pos.z = base_point.point.z;
                obj_pub.publish (pos);
                pub_obj_position = true;
            }
            
        }
    }

public:
    int run (int argc, char** argv)
    {
        ROS_INFO("----------INIT----------");
        ros::init (argc, argv, "object_detection");
        ros::NodeHandle nh;
        ROS_INFO("----Waiting for image----");
        //----------------------------------------------------
        nh.param<std::string>("sub_pcl_topic_name",        sub_pcl_topic_name,        "/astra/depth/points");
        nh.param<std::string>("sub_image_raw_topic_name",  sub_image_raw_topic_name,  "/astra/rgb/image_raw");
        nh.param<std::string>("pub_object_pos_topic_name", pub_object_pos_topic_name, "/kamerider_image/object_position_base");
        

        pcl_sub = nh.subscribe(sub_pcl_topic_name, 1, &object_detection::pclCallback, this);
        img_sub = nh.subscribe(sub_image_raw_topic_name, 1, &object_detection::imageCallback, this);  
        obj_pub = nh.advertise<move_and_grasp::ObjectPosition>(pub_object_pos_topic_name, 1);        
        std::cout << "Receving message from topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << sub_pcl_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "Publishing message to topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << pub_object_pos_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;
        cv::namedWindow("find_object_window");
        cv::namedWindow("object_detection");
        cv::setMouseCallback("find_object_window", on_mouse, &cv_image);
        
        ros::spin();
    }

};

int main(int argc, char** argv)
{
    object_detection detector;
    return detector.run(argc, argv);
}

