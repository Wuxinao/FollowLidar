//
// Created by localization on 11/4/19.
//

#pragma once

#include <thread>
// For ROS output.
#include <cstdint>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/GridCells.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	 CLS_GridMapLocalization   											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
class CLS_GridMapLocalization // GridMapLocalization
{
    /*-------------------------------------------------*\
    |	Constructor & Destructor   						|
    \*-------------------------------------------------*/
public:
    CLS_GridMapLocalization(ros::NodeHandle &ent_pnh);
    ~CLS_GridMapLocalization();
    /*-------------------------------------------------*\
    |	Subscribe      		            				|
    \*-------------------------------------------------*/
public:
    enum pbl_LIST_Project_Name
    {
        PROJECT_Baoxin,
        PROJECT_Hexi
    };
    pbl_LIST_Project_Name pbl_List_Project;

    std::string pbl_strTopicName_Cloud;
    std::string pbl_strTopicName_Gps;
    std::string pbl_strTopicName_Gps_Heading;
    /*-------------------------------------------------*\
    |	Publish       		            				|
    \*-------------------------------------------------*/
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pbl_Publish_Cloud;

    struct pbl_STU_Pose2DStamped
    {
        double x;
        double y;
        double phi;
        double timestamp;
        pbl_STU_Pose2DStamped() : x(0.0), y(0.0), phi(0.0), timestamp(0.0){};
    };
    pbl_STU_Pose2DStamped *pbl_Publish_Match;
    pbl_STU_Pose2DStamped *pbl_Publish_Match_Lasttime;
    std::vector<pbl_STU_Pose2DStamped> pbl_InitPose_Disturbance;

    /*-------------------------------------------------*\
    |	Initialization mode       						|
    \*-------------------------------------------------*/
public:
    enum pbl_LIST_Initialization_MODE
    {
        MODE_Initial_GPS,
        MODE_Initial_SETNUM
    };
    pbl_LIST_Initialization_MODE pbl_List_Initial_Mode;

    /*-------------------------------------------------*\
    |	Current gml status         						|
    \*-------------------------------------------------*/
public:
    enum pbl_LIST_Localization_STATUS
    {
        // STATUS_WAITING,
        STATUS_INITIAL,
        STATUS_MATCHING
    };
    pbl_LIST_Localization_STATUS pbl_List_Current_Status;

    /*-------------------------------------------------*\
    |	Grid map              							|
    |	Function: pbl_fnc_InitGridMap					|
    |	Input: ent_strMapFolder							|
    |	Output: prv_vec_Gridmap                         |
    \*-------------------------------------------------*/
public:
    void pbl_fnc_InitialGridMap(std::string &ent_strMapFolder);

    struct prv_STU_pntXYI
    {
        int x;
        int y;
        char nValue;
    };
    struct prv_STU_Map
    {
        cv::Point2i pntOrigin;
        std::vector<prv_STU_pntXYI> set_pntXYI;
    };
    std::vector<prv_STU_Map> pbl_vec_Gridmap; // gridmap/cloud_1234567_-123456_0000000.png

    /*-------------------------------------------------*\
    |	Cloud filter                   					|
    \*-------------------------------------------------*/
public:
    void pbl_fnc_CloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &ent_cloud);

private:
    double prv_lfMaxZ, prv_lfMinZ;
    double prv_lfVoxelFilter;

    /*-------------------------------------------------*\
    |	Update map                 	    				|
    \*-------------------------------------------------*/
public:
    nav_msgs::OccupancyGrid pbl_GridMap2D;

private:
    std::thread prv_thread_MapUpdate;
public:
    int prv_map_center_x_;
    int prv_map_center_y_;

public:
    void prv_fnc_UpdateGridMap();
    void prv_fnc_UpdateGridMap_NewTest();

    /*-------------------------------------------------*\
    |	IcpMatch                                        |
    \*-------------------------------------------------*/
public:
    bool pbl_fnc_IcpMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ent_cloud);

    inline double pbl_fnc_GetTimeInterval(clock_t start_time)
    {
        clock_t current_time = clock();
        return (double)(current_time - start_time) / CLOCKS_PER_SEC;
    }

    bool isvalid_match_result(pbl_STU_Pose2DStamped *match_result)
    {
        return (fabs(match_result->x - pbl_Publish_Match->x) < 2 && \
        fabs(match_result->y - pbl_Publish_Match->y) < 2 && \
        fabs(match_result->phi - pbl_Publish_Match->phi) < 2);
    }
    pbl_STU_Pose2DStamped caculate_match_result(pbl_STU_Pose2DStamped *match_result)
    {
        pbl_STU_Pose2DStamped valid_match_result;
        pbl_STU_Pose2DStamped l = *pbl_Publish_Match;
            // l.x = pbl_Publish_Match->x;
            // l.y = pbl_Publish_Match->y;
            // l.phi = pbl_Publish_Match->phi;
        pbl_STU_Pose2DStamped ll = *pbl_Publish_Match_Lasttime;
        if (ll.timestamp < 1)
        {
            valid_match_result.x = match_result->x;
            valid_match_result.y = match_result->y;
            valid_match_result.phi = match_result->phi;
            valid_match_result.timestamp = match_result->timestamp;
            return valid_match_result;          
        }
            // ll.x = pbl_Publish_Match_Lasttime->x;
            // ll.y = pbl_Publish_Match_Lasttime->y;
            // ll.phi = pbl_Publish_Match_Lasttime->phi;
        double theta = 0.0, l_ll_x = 0.0;
        double distance = pow( (pow((l.x - ll.x), 2) + pow((l.y - ll.y), 2)), 0.5);
        if (distance < 1e-6)
        {
            valid_match_result.x = l.x;
            valid_match_result.y = l.y;
            valid_match_result.phi = 2 * l.phi - ll.phi;
            valid_match_result.timestamp = match_result->timestamp;
            return valid_match_result;
        }

        if (fabs(l.y - ll.y) < 1e-6)
        {
            if (l.x >= ll.x)
                l_ll_x = 0;
            else
                l_ll_x = M_PI;
        }
        else
        {
            l_ll_x = acos((l.x - ll.x) / distance);
            if (l.y < ll.y)
                l_ll_x = -l_ll_x;
        }

        theta = l.phi - ll.phi + l_ll_x;

        valid_match_result.x = l.x +  distance * cos(theta);
        valid_match_result.y = l.y +  distance * sin(theta);
        valid_match_result.phi = 2 * l.phi - ll.phi;
        valid_match_result.timestamp = match_result->timestamp;

        return valid_match_result;
            // valid_match_result.x = 2 * pbl_Publish_Match->x - pbl_Publish_Match_Lasttime->x; 
            // valid_match_result.y = 2 * pbl_Publish_Match->y - pbl_Publish_Match_Lasttime->y;
            // valid_match_result.phi = 2 * pbl_Publish_Match->phi - pbl_Publish_Match_Lasttime->phi;
            // valid_match_result.timestamp = match_result->timestamp;

    }

private:
    template <class T>
    inline T prv_fnc_wrapToPi(T a)
    {
        while (a < static_cast<T>(-M_PI))
            a += static_cast<T>(2.0 * M_PI);
        a = fmod(a, static_cast<T>(2.0 * M_PI));
        if (a > static_cast<T>(M_PI))
            a -= static_cast<T>(2.0 * M_PI);
        return a;
    }
    /*-------------------------------------------------*\
    |	my_determineMatching2D                          |
    \*-------------------------------------------------*/
private:
    struct prv_STU_CloudMap
    {
        std::vector<float> x_vec;
        std::vector<float> y_vec;
        std::vector<float> z_vec;
    };
    struct prv_STU_MapSector
    {
        prv_STU_CloudMap map_sector_data;
        double middle_angle, start_angle, end_angle;
        double angle_step;
    };
    prv_STU_MapSector map_sector_1piece;
    struct prv_stu_MatchingPair
    {
        unsigned int Map_idx;
        unsigned int Cld_idx;
        float Map_x, Map_y, Map_z;
        float Cld_x, Cld_y, Cld_z;
        float errorSquareAfterTransformation;
    };
    struct prv_STU_ICP_Params_Fixed
    {
        // fixed parameters
        unsigned int nMaxIterations;
        unsigned int nPointDecimation;
        float fMinAbsStep_trans;
        float fMinAbsStep_rot;
        float fShrinkRatio;
        float fMinThresholdDist;
    };
    prv_STU_ICP_Params_Fixed prv_ICP_Params_Fixed;
    struct prv_STU_ICP_Params_Dynamic
    {
        // dynamic parameters
        float fMaxDistForCorres;
        float fMaxAnglForCorres;
        unsigned int nPointOffset;
    };
    prv_STU_ICP_Params_Dynamic prv_ICP_Params_Dynamic;

    void my_determineMatching2D(
        const prv_STU_CloudMap &ent_CloudMatch,
        const pbl_STU_Pose2DStamped *ent_Cloud_Pose,
        std::vector<prv_stu_MatchingPair> &exp_vec_MathingPair,
        const prv_STU_ICP_Params_Dynamic &ent_ICP_Dynamic);

    bool Check_Point_In_Sector(double angle_rad);

public:
    void prv_fnc_CutSectorMap(double middle_angle);
    // bool pbl_fnc_IcpMatch_Map2Map(void);
    bool pbl_fnc_IcpMatch_Map2Map(pbl_STU_Pose2DStamped *pose_disturbance);

    /*-------------------------------------------------*\
    |	prv_se2_l2                                      |
    \*-------------------------------------------------*/
private:
    bool prv_se2_l2(
        std::vector<prv_stu_MatchingPair> &ent_vec_MathingPair,
        pbl_STU_Pose2DStamped *exp_transformation);
//for test

public:
    std::vector<prv_stu_MatchingPair> pbl_MatchingPair;
    std::vector<std::vector<prv_stu_MatchingPair>> vec_Yang_MatchingPair;
    std::vector<pbl_STU_Pose2DStamped> pbl_vec_Yang_MatchResult;
    std::vector<int> n_vec_PointOffset;
    int nStep;
};


//utm origin
extern double utm_east;
extern double utm_north;

extern FILE *fp_odometry;
extern std::vector<double> scores;