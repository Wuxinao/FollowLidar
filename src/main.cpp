#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "cyber_msgs/Heading.h"
// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// ros-pcl pcl-ros convert
#include <pcl_conversions/pcl_conversions.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// GPS-UTM Convert
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <nav_msgs/GridCells.h>


#include "grid_map_localization.h"

bool isHeadInitial = false;
bool isGPSInitial = false;
int  initial_cnt = 3;

FILE *fp_score;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	GPS          							    				    	█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::Callback_Subscribe_GPS(const sensor_msgs::NavSatFixConstPtr &fix_msg)
{
    if (
        pbl_List_Initial_Mode == CLS_GridMapLocalization::MODE_Initial_GPS &&
        pbl_List_Current_Status == CLS_GridMapLocalization::STATUS_INITIAL)
    {
        geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
        gps_msg->header = fix_msg->header;
        gps_msg->position.latitude = fix_msg->latitude;
        gps_msg->position.longitude = fix_msg->longitude;
        gps_msg->position.altitude = fix_msg->altitude;

        geodesy::UTMPoint utm_point;
        geodesy::fromMsg(gps_msg->position, utm_point);

        static int initial_cnt_copy = initial_cnt;
        // printf("%lf %lf\n", utm_point.easting, utm_point.northing);
        pbl_Publish_Match->timestamp = fix_msg->header.stamp.toSec();

        if (!initial_cnt_copy)
        {
            if (!isGPSInitial)
            {
                isGPSInitial = true;
                pbl_Publish_Match->x /= initial_cnt;
                pbl_Publish_Match->y /= initial_cnt;
            }
        }
        else
        {
            initial_cnt_copy--;
            pbl_Publish_Match->x += utm_point.easting - utm_east-25;
            pbl_Publish_Match->y += utm_point.northing - utm_north-25;
        }

        if (isHeadInitial && isGPSInitial)
            pbl_List_Current_Status = CLS_GridMapLocalization::STATUS_MATCHING;
    }
}
/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	GPS heading   							    				        █
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::Callback_Subscribe_GPS_Head(const cyber_msgs::HeadingConstPtr &heading_msg)
{
    // printf("here H!\n");
    if (
        pbl_List_Initial_Mode == CLS_GridMapLocalization::MODE_Initial_GPS &&
        pbl_List_Current_Status == CLS_GridMapLocalization::STATUS_INITIAL)
    {
        double yaw = heading_msg->data;
        if (yaw >= M_PI)
            yaw -= 2 * M_PI;
        else if (yaw <= -M_PI)
            yaw += 2 * M_PI;
        
        static int initial_cnt_copy = initial_cnt;
        if (!initial_cnt_copy)
        {
            if (!isHeadInitial)
            {
                isHeadInitial = true;
                pbl_Publish_Match->phi /= initial_cnt;
            }
        }
        else
        {
            initial_cnt_copy--;
            pbl_Publish_Match->phi += yaw;
            printf("cnt:%d, yaw = %f\n", initial_cnt_copy, yaw);
        }
    }
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Point Cloud 							    					    █
█	Input:  -															█
█	Output: localization_result											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::Callback_Subscribe_PointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    /*-------------------------------------------------*\
    |	Cloud filter                   					|
    \*-------------------------------------------------*/
    pbl_fnc_CloudFilter(cloud);

    if (pbl_List_Current_Status == CLS_GridMapLocalization::STATUS_MATCHING)
    {
        pbl_mutex_isMatching.lock();
        pbl_Publish_Match->timestamp = cloud_msg->header.stamp.toSec();

        clock_t matching_starttime = clock();
        double start = ros::Time::now().toSec();

        pbl_fnc_IcpMatch(cloud);
        double end = ros::Time::now().toSec();
        // printf("time interval: %f\n", end - start);

        pbl_mutex_isMatching.unlock();

        pbl_Publish_Cloud = cloud;
        /*-------------------------------------------------*\
        |	rviz show localization result              		|
        \*-------------------------------------------------*/
        nav_msgs::Odometry OutResultMsg;
        OutResultMsg.pose.pose.position.x = pbl_Publish_Match->x;
        OutResultMsg.pose.pose.position.y = pbl_Publish_Match->y;
        OutResultMsg.pose.pose.position.z = 0;
        OutResultMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pbl_Publish_Match->phi);
        OutResultMsg.header.frame_id = "map";
        OutResultMsg.header.stamp = ros::Time().fromSec(pbl_Publish_Match->timestamp);
        publisher_Odometry_LocalizationResult->publish(OutResultMsg);

        // printf("time interval: %lf\n", pbl_fnc_GetTimeInterval(matching_starttime));

    }
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Publish point cloud     						   					█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::Callback_Publish_PointCloud(const ros::TimerEvent &)
{
    
    if (pbl_Publish_Match != nullptr)
    {
        registered_transform->stamp_ = ros::Time().fromSec(pbl_Publish_Match->timestamp);
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0,
                                                                                 pbl_Publish_Match->phi);
        registered_transform->setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
        registered_transform->setOrigin(
            tf::Vector3(pbl_Publish_Match->x, pbl_Publish_Match->y, 0));
        tf_broadcaster->sendTransform(*registered_transform);
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pbl_Publish_Cloud, cloud_msg);
        cloud_msg.header.frame_id = "lidar";
        cloud_msg.header.stamp = ros::Time().fromSec(pbl_Publish_Match->timestamp);
        publisher_PointCloud2_Lidar->publish(cloud_msg);
    }
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Publish grid map         						   					█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::Callback_Publish_GridMap(const ros::TimerEvent &)
{
    // nav_msgs::OccupancyGrid map_msg;
    pbl_mutex_isMatching.lock();
    if (pbl_GridMap2D.info.width != 0)
    {
        publisher_OccupancyGrid_Map->publish(pbl_GridMap2D);
    }
    pbl_mutex_isMatching.unlock();
}

void CLS_GridMapLocalization::prv_fnc_LidarEnableControl()
{
    //lidar input control window
    cv::namedWindow("ControlWindow", cv::WINDOW_NORMAL);

    cv::Mat OutputArray(cv::Mat::zeros(200, 400, CV_8UC3));

    bool timingStarted = false;
    ros::Time startTime;
    ros::Time endTime;

    while (ros::ok()) 
    {
        char key = cv::waitKey(10);
        if (key == 'q' || key == 'Q') 
        {
            break;
        }
        else if (key == 't' || key == 'T')
        {
            // Start timing
            timingStarted = true;
            startTime = ros::Time::now();
            std::string text = "Timing now";
            OutputArray = cv::Mat::zeros(200, 400, CV_8UC3);
            cv::putText(OutputArray, text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }
        else if ((key == 'f' || key == 'F') && timingStarted)
        {
            // End timing
            timingStarted = false;
            endTime = ros::Time::now();
            double duration = (endTime - startTime).toSec(); // Convert to milliseconds

            std::string text = "Time Duration: " + std::to_string(duration) + " s";
            OutputArray = cv::Mat::zeros(200, 400, CV_8UC3);
            cv::putText(OutputArray, text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }
        cv::imshow("ControlWindow", OutputArray);
    }
    cv::destroyWindow("ControlWindow");
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "icp_viewer");
    ros::NodeHandle pnh("~");
    CLS_GridMapLocalization *cls_gml = new CLS_GridMapLocalization(pnh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    

    return 0;
}

