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
            cls_gml->pbl_Publish_Match->x = utm_point.easting - 355000.0;
            cls_gml->pbl_Publish_Match->y = utm_point.northing - 2700000.0;

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

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	Matching & rviz                										█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    cls_gml->prv_fnc_UpdateGridMap();
    // cls_gml->pbl_fnc_CloudFilter(ent_cloud);
    // cls_gml->pbl_fnc_IcpMatch(ent_cloud);

    /*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
    █	Visualization               										█
    \*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
    /*-------------------------------------------------*\
    |	GUI                                        		|
    \*-------------------------------------------------*/
    cv::Mat im3Src = cv::Mat::zeros(cv::Size(200, 100), CV_8UC3);
    cv::imshow("Control", im3Src);
    int nKey = 0;
    int nIter = 2;
    int nIterLast = -1;

	double sector_angle_step = 5 * M_PI / 180;
    double middle_angle = 0.0;

    while (nKey != 'q')
    {
        nKey = cv::waitKey(30);
        // if (nKey == 'a' && nIter > 0)
        // {
        //     nIter--;
        // }
        // else if (nKey == 'd' && nIter < (int)cls_gml->pbl_vec_Yang_MatchResult.size()-1)
        // {
        //     nIter++;
        // }
        if (nKey == 'w' || nKey == 's')
        {
            if (nKey == 'w')
                middle_angle += sector_angle_step;
            else
                middle_angle -= sector_angle_step;

            if (middle_angle > 2 * M_PI - sector_angle_step + 1e-5)
            {
                middle_angle -= 2 * M_PI;
            }
            if (middle_angle < 0 - 1e-5)   //0-355 deg
            {
                middle_angle += 2 * M_PI;
            }
            cls_gml->prv_fnc_CutSectorMap(middle_angle);
            cls_gml->pbl_fnc_IcpMatch_Map2Map();

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

            // for (int i = 0; i < 20; i++)     //marker
            // {
            //     for (int j = 0; j < 20; j++)
            //     {
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].x = i * 1.0 / 10;
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].y = j * 1.0 / 10;
            //         cloud_sector->points[cls_gml->pbl_MatchingPair.size() + i * 20 + j].z = 0;
            //     }
            // }

            sensor_msgs::PointCloud2 cloud_msg_sector;
            pcl::toROSMsg(*cloud_sector, cloud_msg_sector);
            cloud_msg_sector.header.frame_id = "lidar";
            cloud_msg_sector.header.stamp = lfTime;

            registered_transform->stamp_ = lfTime;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0,
                                                                                    cls_gml->pbl_Publish_Match->phi); // Yaw
            registered_transform->setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
            registered_transform->setOrigin(
                tf::Vector3(cls_gml->pbl_Publish_Match->x, cls_gml->pbl_Publish_Match->y, 0));
            
            tf_broadcaster->sendTransform(*registered_transform);
            // publish_PointCloud2_Lidar->publish(cloud_msg);
            // publish_PointCloud2_Lidar_Matched->publish(cloud_msg_matched);
            publish_PointCloud2_Lidar_Sector->publish(cloud_msg_sector);   

        }




        // /*-------------------------------------------------*\
        // |	Pointcloud_matched                         		|
        // \*-------------------------------------------------*/
        // ros::Publisher *publish_PointCloud2_Lidar_Matched;
        // publish_PointCloud2_Lidar_Matched = new ros::Publisher(pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud_matched", 1));

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_matched(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_notmatched(new pcl::PointCloud<pcl::PointXYZ>());
        // *cloud_notmatched = *ent_cloud;
        // for (int i = (int)cls_gml->vec_Yang_MatchingPair[nIter].size() - 1; i >= 0; i--)
        // {
        //     cloud_notmatched->erase(cloud_notmatched->begin() + cls_gml->vec_Yang_MatchingPair[nIter][i].Cld_idx);
        //     cloud_matched->points.push_back(ent_cloud->points[cls_gml->vec_Yang_MatchingPair[nIter][i].Cld_idx]);
        // }
        // sensor_msgs::PointCloud2 cloud_msg_matched;
        // pcl::toROSMsg(*cloud_matched, cloud_msg_matched);
        // cloud_msg_matched.header.frame_id = "lidar";
        // cloud_msg_matched.header.stamp = lfTime;

        // /*-------------------------------------------------*\
        // |	Pointcloud                                 		|
        // \*-------------------------------------------------*/
        // if (nIter != nIterLast)
        // {
        //     nIterLast = nIter;
        //     printf("%d\n", nIter);
        // }

        // ros::Publisher *publish_PointCloud2_Lidar;
        // publish_PointCloud2_Lidar = new ros::Publisher(pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud", 1));
        // registered_transform->stamp_ = lfTime;
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0,
        //                                                                          cls_gml->pbl_vec_Yang_MatchResult[nIter].phi); // Yaw
        // registered_transform->setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
        // registered_transform->setOrigin(
        //     tf::Vector3(cls_gml->pbl_vec_Yang_MatchResult[nIter].x, cls_gml->pbl_vec_Yang_MatchResult[nIter].y, 0));

        // sensor_msgs::PointCloud2 cloud_msg;
        // pcl::toROSMsg(*cloud_notmatched, cloud_msg);
        // cloud_msg.header.frame_id = "lidar";
        // cloud_msg.header.stamp = lfTime;

        
        /*-------------------------------------------------*\
        |	Map                                      		|
        \*-------------------------------------------------*/
        ros::Publisher *publish_map = new ros::Publisher(pnh.advertise<nav_msgs::OccupancyGrid>("grid_map_new", 1));
        // /*-------------------------------------------------*\
        // |	Odometry                                 		|
        // \*-------------------------------------------------*/
        // ros::Publisher *publish_odom_result = new ros::Publisher(pnh.advertise<nav_msgs::Odometry>("map_matching_result/utm_new", 5));
        // nav_msgs::Odometry OutResultMsg;
        // OutResultMsg.pose.pose.position.x = cls_gml->pbl_vec_Yang_MatchResult[nIter].x;
        // OutResultMsg.pose.pose.position.y = cls_gml->pbl_vec_Yang_MatchResult[nIter].y;
        // OutResultMsg.pose.pose.position.z = 0;
        // OutResultMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, cls_gml->pbl_vec_Yang_MatchResult[nIter].phi);
        // OutResultMsg.header.frame_id = "map";
        // OutResultMsg.header.stamp = lfTime;

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

        // /*-------------------------------------------------*\
        // |	grid status                   		            |
        // \*-------------------------------------------------*/
        // ros::Publisher *publish_GridCells_Map_Matched = new ros::Publisher(pnh.advertise<nav_msgs::GridCells>("grid_cell", 0.01));
        // nav_msgs::GridCells pbl_GridCells_Map_Matched;
        // pbl_GridCells_Map_Matched.header.frame_id = "map";
        // pbl_GridCells_Map_Matched.cell_height = 0.4;
        // pbl_GridCells_Map_Matched.cell_width = 0.4;

        // pbl_GridCells_Map_Matched.cells.clear();
      

        // // check matching pair to update the matching status on the matched grids
        // for (int k = 0; k < cls_gml->vec_Yang_MatchingPair[nIter].size(); k++)
        // {
        //     for (int i = 0; i < cls_gml->pbl_vec_Gridmap.size(); i++)
        //     {
        //         double lfThisX = 10 * cls_gml->pbl_GridMap2D.info.origin.position.x + cls_gml->vec_Yang_MatchingPair[nIter][k].Map_idx % cls_gml->pbl_GridMap2D.info.width - 10 * cls_gml->pbl_vec_Gridmap[i].pntOrigin.x + 250.0;
        //         double lfThisY = 10 * cls_gml->pbl_GridMap2D.info.origin.position.y + cls_gml->vec_Yang_MatchingPair[nIter][k].Map_idx / cls_gml->pbl_GridMap2D.info.width - 10 * cls_gml->pbl_vec_Gridmap[i].pntOrigin.y + 250.0;

        //         // check whether the current MatchingPair is on current Gridmap
        //         // continue if not
        //         if (lfThisX > 500 || lfThisY > 500)
        //         {
        //             // printf("hereA! %d %d\n", lfThisX, lfThisY);
        //             continue;
        //         }

        //         bool isPoint = false;
        //         for (int j = 0; j < cls_gml->pbl_vec_Gridmap[i].set_pntXYI.size(); j++)
        //         {
        //             if (cls_gml->pbl_vec_Gridmap[i].set_pntXYI[j].x == lfThisX && cls_gml->pbl_vec_Gridmap[i].set_pntXYI[j].y == lfThisY)
        //             {
        //                 geometry_msgs::Point temp;
        //                 temp.x = cls_gml->pbl_vec_Gridmap[i].pntOrigin.x + cls_gml->pbl_vec_Gridmap[i].set_pntXYI[j].x / 10.0 - 25.0 + 0.05;
        //                 temp.y = cls_gml->pbl_vec_Gridmap[i].pntOrigin.y + cls_gml->pbl_vec_Gridmap[i].set_pntXYI[j].y / 10.0 - 25.0 + 0.05;
        //                 temp.z = 0;
        //                 pbl_GridCells_Map_Matched.cells.push_back(temp);
        //                 isPoint = true;
        //                 break;
        //             }
        //         }
        //         if (isPoint)
        //             break;
        //     }
        // }
        // tf_broadcaster->sendTransform(*registered_transform);
        // // publish_PointCloud2_Lidar->publish(cloud_msg);
        // // publish_PointCloud2_Lidar_Matched->publish(cloud_msg_matched);
        // publish_PointCloud2_Lidar_Sector->publish(cloud_msg_sector);
        
        publish_map->publish(cls_gml->pbl_GridMap2D);
        publish_odom_result->publish(OutResultMsg);
        // publish_GridCells_Map_Matched->publish(pbl_GridCells_Map_Matched);
    }

    return 0;
}