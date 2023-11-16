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

FILE *fp_score;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "icp_viewer");
    ros::NodeHandle pnh("~");
    CLS_GridMapLocalization *cls_gml = new CLS_GridMapLocalization(pnh);

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	inputs for ICP                   									█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr ent_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // nav_msgs::OccupancyGrid GridMap2D;

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	TF                                 									█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    tf::TransformBroadcaster *tf_broadcaster = new tf::TransformBroadcaster;
    tf::StampedTransform *registered_transform = new tf::StampedTransform;
    registered_transform->frame_id_ = "map";
    registered_transform->child_frame_id_ = "lidar";

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	Load Pointcloud and GPS        										█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region Load Pointcloud and GPS

    /*-------------------------------------------------*\
    |	input bag                               		|
    \*-------------------------------------------------*/
    std::string strMapFile;
    pnh.param<std::string>("PRMTR_strMapFile", strMapFile, "");
    std::string input_bag_file = strMapFile;

    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    rosbag::View view(input_bag);

    /*-------------------------------------------------*\
    |	load bag                                  		|
    \*-------------------------------------------------*/
    bool isFirstCloud = true;
    bool isFirstGPS = true;
    bool isFirstHeading = true;
    for (const rosbag::MessageInstance &m : view)
    {
        // if (m.getTopic() == "/driver/livox/point_cloud" && isFirstCloud)
        // {
        //     sensor_msgs::PointCloud2::Ptr point_msg = m.instantiate<sensor_msgs::PointCloud2>();

        //     pcl::fromROSMsg(*point_msg, *ent_cloud);
        //     printf("%d\n", ent_cloud->points.size());
        //     isFirstCloud = false;
        // }
        if (m.getTopic() == "/driver/gps/fix" && isFirstGPS)
        {
            sensor_msgs::NavSatFix::Ptr gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
            geographic_msgs::GeoPointStampedPtr geo_msg(new geographic_msgs::GeoPointStamped());
            geo_msg->header = gps_msg->header;
            geo_msg->position.latitude = gps_msg->latitude;
            geo_msg->position.longitude = gps_msg->longitude;
            geo_msg->position.altitude = gps_msg->altitude;

            geodesy::UTMPoint utm_point;
            geodesy::fromMsg(geo_msg->position, utm_point);
            cls_gml->pbl_Publish_Match->timestamp = gps_msg->header.stamp.toSec();
            // cls_gml->pbl_Publish_Match->x = utm_point.easting - 355000;
            // cls_gml->pbl_Publish_Match->y = utm_point.northing - 2700000;
            cls_gml->pbl_Publish_Match->x = utm_point.easting - 334700;
            cls_gml->pbl_Publish_Match->y = utm_point.northing - 2692400;

            isFirstGPS = false;
        }
        // else if (m.getTopic() == "/strong/heading" && isFirstHeading)
        // {
        //     cyber_msgs::Heading::Ptr heading_msg = m.instantiate<cyber_msgs::Heading>();
        //     double yaw = heading_msg->data;
        //     if (yaw >= M_PI)
        //         yaw -= 2 * M_PI;
        //     else if (yaw <= -M_PI)
        //         yaw += 2 * M_PI;
        //     cls_gml->pbl_Publish_Match->phi = yaw + 0.05;
        //     isFirstHeading = false;
        // }
    }
#pragma endregion

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	Generate Localmap              										█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    ros::Time lfTime = ros::Time::now();

    /*-------------------------------------------------*\
    |	Map                                      		|
    \*-------------------------------------------------*/
    cls_gml->prv_fnc_UpdateGridMap();
    // ros::Publisher *publish_map = new ros::Publisher(pnh.advertise<nav_msgs::OccupancyGrid>("grid_map_new", 1));
    // publish_map->publish(cls_gml->pbl_GridMap2D);


    // /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    // █	Visualization               										█
    // \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    // /*-------------------------------------------------*\
    // |	GUI                                        		|
    // \*-------------------------------------------------*/
    // cv::Mat im3Src = cv::Mat::zeros(cv::Size(200, 100), CV_8UC3);
    // cv::imshow("Control", im3Src);
    // int nKey = 0;
    // int nIter = 2;
    // int nIterLast = -1;

	double sector_angle_step = 5 * M_PI / 180;
    double middle_angle = 0.0;

    // while (nKey != 'q')
    // {
        // nKey = cv::waitKey(30);
        // if (nKey == 'a' && nIter > 0)
        // {
        //     nIter--;
        // }
        // else if (nKey == 'd' && nIter < (int)cls_gml->pbl_vec_Yang_MatchResult.size()-1)
        // {
        //     nIter++;
        // }
        // if (nKey == 'w' || nKey == 's')
        // {

	fp_score = fopen("/home/w/502D/NewLoc2D_ws/score.txt", "a");

    for (int i = 0; i < cls_gml->pbl_InitPose_Disturbance.size(); i++)
    {
        //publish initial pose (disturbed)
        ros::Publisher *publish_disturb = new ros::Publisher(pnh.advertise<nav_msgs::Odometry>("map_matching_result/utm_new_disturb", 5));
        nav_msgs::Odometry OutResultMsg_init;
        OutResultMsg_init.pose.pose.position.x = cls_gml->pbl_InitPose_Disturbance[i].x + (cls_gml->prv_map_center_x_ - 25);
        OutResultMsg_init.pose.pose.position.y = cls_gml->pbl_InitPose_Disturbance[i].y + (cls_gml->prv_map_center_y_ - 25);
        OutResultMsg_init.pose.pose.position.z = 0;
        OutResultMsg_init.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, cls_gml->pbl_InitPose_Disturbance[i].phi);
        OutResultMsg_init.header.frame_id = "map";
        OutResultMsg_init.header.stamp = lfTime;
        publish_disturb->publish(OutResultMsg_init);
        scores.clear();

        for (int j = 0; j < 2 * M_PI / sector_angle_step; j++)
        {
            middle_angle = j * sector_angle_step;
            cls_gml->prv_fnc_CutSectorMap(middle_angle);
            fprintf(fp_odometry, "%d\t", i);
            cls_gml->pbl_fnc_IcpMatch_Map2Map(&(cls_gml->pbl_InitPose_Disturbance[i]));
    
            /*-------------------------------------------------*\
            |	Pointcloud_sector                        		|
            \*-------------------------------------------------*/
            ros::Publisher *publish_PointCloud2_Lidar_Sector;
            publish_PointCloud2_Lidar_Sector = new ros::Publisher(pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud_sector", 1));

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sector(new pcl::PointCloud<pcl::PointXYZ>());
            // cloud_sector->points.resize(cls_gml->pbl_MatchingPair.size()+400); //marker
            cloud_sector->points.resize(cls_gml->pbl_MatchingPair.size());
            for (int i = 0; i < cls_gml->pbl_MatchingPair.size(); i++)
            {
                cloud_sector->points[i].x = cls_gml->pbl_MatchingPair[i].Cld_x;
                cloud_sector->points[i].y = cls_gml->pbl_MatchingPair[i].Cld_y;
                cloud_sector->points[i].z = cls_gml->pbl_MatchingPair[i].Cld_z;
            }
            sensor_msgs::PointCloud2 cloud_msg_sector;
            pcl::toROSMsg(*cloud_sector, cloud_msg_sector);
            cloud_msg_sector.header.frame_id = "lidar";
            cloud_msg_sector.header.stamp = lfTime;

            registered_transform->stamp_ = lfTime;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, cls_gml->pbl_Publish_Match->phi); // Yaw
            // printf("phi = %.3f\n", cls_gml->pbl_Publish_Match->phi * 180 / M_PI);
            registered_transform->setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
            registered_transform->setOrigin(
                tf::Vector3(cls_gml->pbl_Publish_Match->x, cls_gml->pbl_Publish_Match->y, 0));
            
            tf_broadcaster->sendTransform(*registered_transform);
            // publish_PointCloud2_Lidar->publish(cloud_msg);
            // publish_PointCloud2_Lidar_Matched->publish(cloud_msg_matched);
            publish_PointCloud2_Lidar_Sector->publish(cloud_msg_sector);   

            /*-------------------------------------------------*\
            |	Odometry                                 		|
            \*-------------------------------------------------*/
            ros::Publisher *publish_odom_result = new ros::Publisher(pnh.advertise<nav_msgs::Odometry>("map_matching_result/utm_new", 5));
            nav_msgs::Odometry OutResultMsg;
            OutResultMsg.pose.pose.position.x = cls_gml->pbl_Publish_Match->x;
            OutResultMsg.pose.pose.position.y = cls_gml->pbl_Publish_Match->y;
            OutResultMsg.pose.pose.position.z = 0;
            OutResultMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, cls_gml->pbl_Publish_Match->phi);
            OutResultMsg.header.frame_id = "map";
            OutResultMsg.header.stamp = lfTime;
            publish_odom_result->publish(OutResultMsg);

            ros::Publisher *publish_map = new ros::Publisher(pnh.advertise<nav_msgs::OccupancyGrid>("grid_map_new", 1));
            publish_map->publish(cls_gml->pbl_GridMap2D);

            // ros::Duration(0.5).sleep();
        }

        int l = std::min_element(scores.begin(), scores.end()) - scores.begin();
        fprintf(fp_score, "disturb index: %d, best angle: %.1f, score: %f\n", i, l * sector_angle_step * 180 / M_PI, scores[l]);
        scores.clear();

    }
            // if (nKey == 'w')
            //     middle_angle += sector_angle_step;
            // else
            //     middle_angle -= sector_angle_step;

            // if (middle_angle > 2 * M_PI - sector_angle_step + 1e-5)
            // {
            //     middle_angle -= 2 * M_PI;
            // }
            // if (middle_angle < 0 - 1e-5)   //0-355 deg
            // {
            //     middle_angle += 2 * M_PI;
            // }
            // cls_gml->prv_fnc_CutSectorMap(middle_angle);
            // cls_gml->pbl_fnc_IcpMatch_Map2Map(cls_gml->pbl_InitPose_Disturbance[i]);


            // for (int i = 0; i < 20; i++)     //marker
            // {
            //     for (int j = 0; j < 20; j++)
            //     {
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].x = i * 1.0 / 10;
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].y = j * 1.0 / 10;
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].z = 0;
            //     }
            // }

        // }

        // publish_GridCells_Map_Matched->publish(pbl_GridCells_Map_Matched);
    // }

    return 0;
}

