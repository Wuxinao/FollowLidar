/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Lidar Localization 							    					█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

#include "grid_map_localization.h"
#include "boost/filesystem.hpp"
#include "time.h"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>


double utm_east = 0.0;
double utm_north = 0.0;
// std::vector<float> RWeight;
FILE *fp_odometry;
std::vector<double> scores;
ros::Timer map_publishing_timer;
ros::Timer point_cloud_publishing_timer;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Grid Map Localization Constructor			    					█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
CLS_GridMapLocalization::CLS_GridMapLocalization(ros::NodeHandle &ent_pnh)
{
	/*-------------------------------------------------*\
	|	Initialization                    				|
	\*-------------------------------------------------*/
	pbl_List_Current_Status = STATUS_INITIAL;

	prv_map_center_x_ = INT_MAX;
	prv_map_center_y_ = INT_MAX;

	pbl_Publish_Match = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;
	pbl_Publish_Match_Lasttime = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;
	pbl_Publish_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
	// fp_odometry = fopen("/home/w/502D/NewLoc2D_ws/odometry.txt", "a");

	// init pose disturbance
	pbl_InitPose_Disturbance.clear();
	for (int i = 0; i < 9; i++)
	{
		pbl_STU_Pose2DStamped pose_disturbance;
		pose_disturbance.x = (i % 3 - 1) * 0.1;
		pose_disturbance.y = - (i / 3 - 1) * 0.1;
		if (i % 4 == 2)
			pose_disturbance.phi = - 5 * M_PI / 180;
		else if (i == 0 || i == 8)
			pose_disturbance.phi = 5 * M_PI / 180;
		else
			pose_disturbance.phi = 0;

		pbl_InitPose_Disturbance.push_back(pose_disturbance);
	}

	// from launch file
	// Set project name
	int nProjectID;
	ent_pnh.param<int>("PRMTR_nProjectID", nProjectID, 0);
	pbl_List_Project = static_cast<CLS_GridMapLocalization::pbl_LIST_Project_Name>(nProjectID);
	// Set topic name
	ent_pnh.param<std::string>("PRMTR_strTopicName_Cloud", pbl_strTopicName_Cloud, "");
	ent_pnh.param<std::string>("PRMTR_strTopicName_GPS", pbl_strTopicName_Gps, "");
	ent_pnh.param<std::string>("PRMTR_strTopicName_GPS_Heading", pbl_strTopicName_Gps_Heading, "");

	// Set map folder
	std::string strMapFolder;
	ent_pnh.param<std::string>("PRMTR_strMapFolder", strMapFolder, "");
	pbl_fnc_InitialGridMap(strMapFolder);

	// Set height boundaries
	ent_pnh.param<double>("PRMTR_lfMaxZ", prv_lfMaxZ, 5);
	ent_pnh.param<double>("PRMTR_lfMinZ", prv_lfMinZ, 2.5);
	prv_lfVoxelFilter = 0.3;

	//set utm origin
	ent_pnh.param<double>("PRMTR_utmEast", utm_east, 334700.0);
	ent_pnh.param<double>("PRMTR_utmNorth", utm_north, 2692400.0);


	int nInitialMode;
	ent_pnh.param<int>("PRMTR_nInitMode", nInitialMode, 0);
	pbl_List_Initial_Mode = static_cast<CLS_GridMapLocalization::pbl_LIST_Initialization_MODE>(nInitialMode);

	/*-------------------------------------------------*\
    |	Subscribers		                         		|
    \*-------------------------------------------------*/
    subscriber_lidar = ent_pnh.subscribe<sensor_msgs::PointCloud2>(pbl_strTopicName_Cloud.c_str(), 1, &CLS_GridMapLocalization::Callback_Subscribe_PointCloud, this);
    subscriber_gps = ent_pnh.subscribe<sensor_msgs::NavSatFix>(pbl_strTopicName_Gps.c_str(), 1000, &CLS_GridMapLocalization::Callback_Subscribe_GPS, this);
    subscriber_gps_yaw = ent_pnh.subscribe<cyber_msgs::Heading>(pbl_strTopicName_Gps_Heading, 1, &CLS_GridMapLocalization::Callback_Subscribe_GPS_Head, this);
	
	/*-------------------------------------------------*\
    |	Publishers and TF                         		|
    \*-------------------------------------------------*/
	publisher_PointCloud2_Sector = new ros::Publisher(ent_pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud_sector", 1));
	publisher_PointCloud2_Lidar = new ros::Publisher(ent_pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud", 1));
    publisher_OccupancyGrid_Map = new ros::Publisher(ent_pnh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1));
    publisher_Odometry_AssessResult = new ros::Publisher(ent_pnh.advertise<nav_msgs::Odometry>("assessment_result", 5));
    publisher_Odometry_LocalizationResult = new ros::Publisher(ent_pnh.advertise<nav_msgs::Odometry>("localization_result", 5));

	tf_broadcaster = new tf::TransformBroadcaster;
	tf_broadcaster_1 = new tf::TransformBroadcaster;
    registered_transform = new tf::StampedTransform;
	registered_transform->frame_id_ = "map";
    registered_transform->child_frame_id_ = "lidar";
    assessment_transform = new tf::StampedTransform;
	assessment_transform->frame_id_ = "map";
	assessment_transform->child_frame_id_ = "assess";


	/*-------------------------------------------------*\
    |	Publish Grid Map                          		|
    \*-------------------------------------------------*/
    map_publishing_timer = ent_pnh.createTimer(ros::Duration(1), &CLS_GridMapLocalization::Callback_Publish_GridMap, this);

    /*-------------------------------------------------*\
    |	Publish Point Cloud                        		|
    \*-------------------------------------------------*/
    point_cloud_publishing_timer = ent_pnh.createTimer(ros::Duration(0.05), &CLS_GridMapLocalization::Callback_Publish_PointCloud, this);

	/*-------------------------------------------------*\
	|	Update map                 	    				|
	\*-------------------------------------------------*/
	prv_thread_MapUpdate = std::thread(&CLS_GridMapLocalization::prv_fnc_UpdateGridMap, this);

	/*-------------------------------------------------*\
	|	Match Assessment            	    			|
	\*-------------------------------------------------*/
	pbl_thread_MatchAssessment = std::thread(&CLS_GridMapLocalization::prv_fnc_MatchAssessment, this);

	/*-------------------------------------------------*\
	|	Lidar Enable GUI Control      	    			|
	\*-------------------------------------------------*/
	pbl_thread_LidarEnableControl = std::thread(&CLS_GridMapLocalization::prv_fnc_LidarEnableControl, this);

	/*-------------------------------------------------*\
	|	ICP parameters            	    				|
	\*-------------------------------------------------*/
	// fixed parameters
	prv_ICP_Params_Fixed.nMaxIterations = 25;
	prv_ICP_Params_Fixed.nPointDecimation = 6; //12
	prv_ICP_Params_Fixed.fMinAbsStep_trans = 1e-2f; // 1e-6f;
	prv_ICP_Params_Fixed.fMinAbsStep_rot = 1e-3f;	// 1e-6f;
	prv_ICP_Params_Fixed.fShrinkRatio = 0.5f;
	prv_ICP_Params_Fixed.fMinThresholdDist = 0.1f;
	// dynamic parameters
	prv_ICP_Params_Dynamic.fMaxDistForCorres = 0.75f;//0.75;//0.1f;		   // options_thresholdDist;
	prv_ICP_Params_Dynamic.fMaxAnglForCorres = DEG2RAD(0.15f); // options_thresholdAng;
	prv_ICP_Params_Dynamic.nPointOffset = 0;
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Grid Map Localization Destructor			    					█
█	Input:  -															█
█	Output: -           												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
CLS_GridMapLocalization::~CLS_GridMapLocalization()
{
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: pbl_fnc_InitGridMap				    					█
█	Input: ent_strMapFolder												█
█	Output: prv_vec_Gridmap												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::pbl_fnc_InitialGridMap(std::string &ent_strMapFolder)
{

	std::vector<boost::filesystem::path> paths;
	boost::filesystem::path myFolder = ent_strMapFolder;
	if (boost::filesystem::exists(myFolder) && boost::filesystem::is_directory(myFolder))
	{
		for (auto const &entry : boost::filesystem::recursive_directory_iterator(myFolder))
		{
			if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".png")
				paths.emplace_back(entry.path().filename());
		}
	}
	else
	{
		std::cout << "gridmap not found!" << std::endl;
	}
	ROS_INFO("Maps in png form: %d loaded", paths.size());
	for (int i = 0; i < paths.size(); i++)
	{
		std::string strLoadPng = myFolder.string() + "/" + paths[i].string();
		cv::Mat im3Src = cv::imread(strLoadPng);
		std::vector<prv_STU_pntXYI> grid_pushback;
		for (int x = 0; x < im3Src.rows; x++)
		{
			for (int y = 0; y < im3Src.cols; y++)
			{
				unsigned char ucBright = im3Src.at<cv::Vec3b>(499 - y, x)(0);
				if (ucBright != 255 && ucBright != 254)
				{
					float fTemp = 0;//-(100.0 * (float)ucBright / 255.0) + 100.0;
					grid_pushback.push_back({x, y, (char)fTemp});
				}
			}
		}
		pbl_vec_Gridmap.push_back({cv::Point2i(
									   std::stoi(paths[i].string().substr(6, 7)),
									   std::stoi(paths[i].string().substr(14, 7))),
								   grid_pushback});
	}
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: pbl_fnc_CloudFilter				    					█
█	Input: filter parameters											█
█	Output: cloud														█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::pbl_fnc_CloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &ent_cloud)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(ent_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(prv_lfMinZ, prv_lfMaxZ);
	pass.filter(*ent_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-100, 100);
	pass.filter(*ent_cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-100, 100);
	pass.filter(*ent_cloud);
	// Voxelgrid filter
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(ent_cloud);
	sor.setLeafSize(prv_lfVoxelFilter, prv_lfVoxelFilter, prv_lfVoxelFilter);
	sor.filter(*ent_cloud);
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: prv_fnc_UpdateGridMap				    					█
█	Input: pbl_matching_result_											█
█	Output: my_pbl_GridMap2D											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::prv_fnc_UpdateGridMap()
{
	int prv_m_map_size_ = 5;
	unsigned prv_square_size_ = 50;
	double prv_map_resolution_ = 0.1; //
	while (true)
	{
		usleep(100000);

		if (pbl_List_Current_Status != STATUS_MATCHING)
		{
			continue;
		}
		
		int map_center_x_new = prv_square_size_ * ceil((float(pbl_Publish_Match->x) - float(prv_square_size_) / 2) / float(prv_square_size_));
		int map_center_y_new = prv_square_size_ * ceil((float(pbl_Publish_Match->y) - float(prv_square_size_) / 2) / float(prv_square_size_));
		
		if (prv_map_center_x_ != map_center_x_new || prv_map_center_y_ != map_center_y_new)
		{
			prv_map_center_x_ = map_center_x_new;
			prv_map_center_y_ = map_center_y_new;
			std::cout << "[INFO] Map center changed to (" << prv_map_center_x_ << "," << prv_map_center_y_ << ")."
					<< std::endl;

			// 读以center为中心的n×n的图
			float x_min = prv_map_center_x_ - float(prv_square_size_) / 2 * float(prv_m_map_size_);
			float x_max = prv_map_center_x_ + float(prv_square_size_) / 2 * float(prv_m_map_size_);
			float y_min = prv_map_center_y_ - float(prv_square_size_) / 2 * float(prv_m_map_size_);
			float y_max = prv_map_center_y_ + float(prv_square_size_) / 2 * float(prv_m_map_size_);
			int image_size_ = int(float(prv_square_size_) / prv_map_resolution_);

			// mrpt::maps::COccupancyGridMap2D *ogm = new mrpt::maps::COccupancyGridMap2D;
			// ogm->setSize(x_min, x_max, y_min, y_max, prv_map_resolution_);

			nav_msgs::OccupancyGrid my_ogm;
			my_ogm.header.frame_id = "map";

			my_ogm.info.width = std::round((x_max - x_min) / prv_map_resolution_);
			my_ogm.info.height = std::round((y_max - y_min) / prv_map_resolution_);
			my_ogm.info.resolution = prv_map_resolution_;

			my_ogm.info.origin.position.x = x_min;
			my_ogm.info.origin.position.y = y_min;
			my_ogm.info.origin.position.z = 0;

			my_ogm.info.origin.orientation.x = 0;
			my_ogm.info.origin.orientation.y = 0;
			my_ogm.info.origin.orientation.z = 0;
			my_ogm.info.origin.orientation.w = 1;

			my_ogm.data.resize(my_ogm.info.width * my_ogm.info.height);

			int image_around_num = 0;
			clock_t map_loading_start_time = clock();

			// #pragma omp parallel for
			for (int i = 0; i < prv_m_map_size_; ++i)
			{
				//  //          #pragma omp parallel for
				for (int j = 0; j < prv_m_map_size_; ++j)
				{
					int new_center_x = prv_map_center_x_ + prv_square_size_ * (i - prv_m_map_size_ / 2);
					int new_center_y = prv_map_center_y_ + prv_square_size_ * (j - prv_m_map_size_ / 2);

					// mrpt::utils::CImage image_in;
					// image_in.
					bool isMap = false;
					int nMapSelected = 0;
					for (int nMapIndex = 0; nMapIndex < pbl_vec_Gridmap.size(); nMapIndex++)
					{
						if (pbl_vec_Gridmap[nMapIndex].pntOrigin == cv::Point2i(new_center_x, new_center_y))
						{
							nMapSelected = nMapIndex;
							isMap = true;
							break;
						}
					}
					if (isMap)
					{
						for (int nPnt = 0; nPnt < pbl_vec_Gridmap[nMapSelected].set_pntXYI.size(); nPnt++)
						{
							int square_index_x = pbl_vec_Gridmap[nMapSelected].set_pntXYI[nPnt].x + i * image_size_;
							int square_index_y = pbl_vec_Gridmap[nMapSelected].set_pntXYI[nPnt].y + j * image_size_;
							// float square_index_value = prv_vec_Gridmap[nMapSelected].set_pntXYI[nPnt].fValue;
							char square_index_value_4show = pbl_vec_Gridmap[nMapSelected].set_pntXYI[nPnt].nValue;
							// ogm->setCell(square_index_x,
							// 			 square_index_y,
							// 			 square_index_value);
							// my_ogm_show.data[my_ogm_show.info.height * square_index_y + square_index_x] = (1.0 - square_index_value) * 100.0;
							my_ogm.data[my_ogm.info.height * square_index_y + square_index_x] = 80; // square_index_value_4show;
							// printf("%d ",square_index_value_4show);
						}
						image_around_num++;
					}
				}
			}

			std::cout << "[INFO] Map loading time: " << pbl_fnc_GetTimeInterval(map_loading_start_time) * 1000 << " ms."
					<< std::endl; // Map loading timer ends, unit s->ms

			if (image_around_num < prv_m_map_size_ * prv_m_map_size_ / 2)
			{
				std::cout << "[WARN] Not enough map image data." << std::endl;
			}

			pbl_mutex_isMatching.lock();
			pbl_GridMap2D = my_ogm;
			pbl_mutex_isMatching.unlock();
		}

	}

}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: prv_fnc_UpdateGridMap2			    					█
█	Input: pbl_matching_result_											█
█	Output: my_pbl_GridMap2D											█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::prv_fnc_UpdateGridMap_NewTest()
{


}

void CLS_GridMapLocalization::prv_fnc_MatchAssessment()
{
	while (!isGPSInitial || pbl_GridMap2D.info.width == 0);
	double sector_angle_step = 5 * M_PI / 180;
    double middle_angle = 0.0;

	while (true)
	{
		// sleep(5);
		pbl_mutex_isMatching.lock();
		*pbl_Publish_Match_Lasttime = *pbl_Publish_Match;
		pbl_mutex_isMatching.unlock();
		ros::Time timeStamp = ros::Time(pbl_Publish_Match_Lasttime->timestamp);
		double t1 = ros::Time::now().toSec();

		//record of all assess results(360 deg)
		scores.clear();
		pbl_Publish_Match_Assessment.clear();

		// for (int i = 0; i < pbl_InitPose_Disturbance.size(); i++)
		// {
		
			for (int j = 0; j < 2 * M_PI / sector_angle_step; j++)
			{
				// timeStamp = ros::Time::now();
				middle_angle = j * sector_angle_step;
				prv_fnc_CutSectorMap(middle_angle);
				// fprintf(fp_odometry, "%d\t", i);
				// printf("%d\t", 4);
				pbl_fnc_IcpMatch_Map2Map(&(pbl_InitPose_Disturbance[4]));
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sector(new pcl::PointCloud<pcl::PointXYZ>());
				cloud_sector->points.resize(pbl_MatchingPair.size() + 25);
				for (int i = 0; i < pbl_MatchingPair.size(); i++)
				{
					cloud_sector->points[i].x = pbl_MatchingPair[i].Cld_x;
					cloud_sector->points[i].y = pbl_MatchingPair[i].Cld_y;
					cloud_sector->points[i].z = pbl_MatchingPair[i].Cld_z;
				}
				for (int i = pbl_MatchingPair.size(); i < pbl_MatchingPair.size() + 25; i++)
				{
					cloud_sector->points[i].x = (i - pbl_MatchingPair.size()) / 5;
					cloud_sector->points[i].x /= 10.0;
					cloud_sector->points[i].y = (i - pbl_MatchingPair.size()) % 5;
					cloud_sector->points[i].y /= 10.0;
					cloud_sector->points[i].z = 0;
				}
				//publish map sector
				sensor_msgs::PointCloud2 cloud_msg_sector;
				pcl::toROSMsg(*cloud_sector, cloud_msg_sector);
				cloud_msg_sector.header.frame_id = "assess";
				cloud_msg_sector.header.stamp = timeStamp;

				assessment_transform->stamp_ = timeStamp;
				geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pbl_Publish_Match_Assessment[j].phi); // Yaw
				assessment_transform->setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
				assessment_transform->setOrigin(
					tf::Vector3(pbl_Publish_Match_Assessment[j].x, pbl_Publish_Match_Assessment[j].y, 0));
				tf_broadcaster_1->sendTransform(*assessment_transform);
				publisher_PointCloud2_Sector->publish(cloud_msg_sector);
				// usleep(100000);

			}
		double t2 = ros::Time::now().toSec();
		// printf("assess finish. time cost: %fs\n", t2 - t1);
		// }

		//publish best orientation
		int best_score_idx = std::min_element(scores.begin(), scores.end()) - scores.begin();
        nav_msgs::Odometry OdomMsg_AssessResult;
        OdomMsg_AssessResult.pose.pose.position.x = pbl_Publish_Match_Assessment[best_score_idx].x;
        OdomMsg_AssessResult.pose.pose.position.y = pbl_Publish_Match_Assessment[best_score_idx].y;
        OdomMsg_AssessResult.pose.pose.position.z = 0;
        OdomMsg_AssessResult.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, best_score_idx * sector_angle_step);
        OdomMsg_AssessResult.header.frame_id = "map";
        OdomMsg_AssessResult.header.stamp = timeStamp;
        publisher_Odometry_AssessResult->publish(OdomMsg_AssessResult);

	}
}

void CLS_GridMapLocalization::prv_fnc_CutSectorMap(double middle_angle)
{
	double sector_angle_step = 5 * M_PI / 180;
	double sector_angle_range = 70 * M_PI / 180;
	double prv_map_resolution_ = 0.1;
	map_sector_1piece.map_sector_data.x_vec.clear();
	map_sector_1piece.map_sector_data.y_vec.clear();
	map_sector_1piece.map_sector_data.z_vec.clear();

	// for (double middle_angle = 0.0; middle_angle < 2 * M_PI; middle_angle += sector_angle_step)
	// {
		map_sector_1piece.middle_angle = middle_angle;
		map_sector_1piece.start_angle = middle_angle - sector_angle_range / 2;
		map_sector_1piece.end_angle = middle_angle + sector_angle_range / 2;
		for (int nMap = 0; nMap < pbl_vec_Gridmap.size(); nMap++)
		{
			for (int nPnt = 0; nPnt < pbl_vec_Gridmap[nMap].set_pntXYI.size(); nPnt+=5)	//downsample
			{
				// double mappnt_y_to_center = pbl_vec_Gridmap[nMap].pntOrigin.y - prv_map_center_y_ + pbl_vec_Gridmap[nMap].set_pntXYI[nPnt].y * prv_map_resolution_;
				// double mappnt_x_to_center = pbl_vec_Gridmap[nMap].pntOrigin.x - prv_map_center_x_ + pbl_vec_Gridmap[nMap].set_pntXYI[nPnt].x * prv_map_resolution_;
				double mappnt_y_to_center = pbl_vec_Gridmap[nMap].pntOrigin.y - pbl_Publish_Match_Lasttime->y + pbl_vec_Gridmap[nMap].set_pntXYI[nPnt].y * prv_map_resolution_;
				double mappnt_x_to_center = pbl_vec_Gridmap[nMap].pntOrigin.x - pbl_Publish_Match_Lasttime->x + pbl_vec_Gridmap[nMap].set_pntXYI[nPnt].x * prv_map_resolution_;
				if (pow(mappnt_x_to_center, 2) + pow(mappnt_y_to_center, 2) > 80 * 80)	//consider points in 30m
					continue;
				double mappnt_angle_to_center = atan2(mappnt_y_to_center, mappnt_x_to_center);
				if (Check_Point_In_Sector(mappnt_angle_to_center))
				{
					map_sector_1piece.map_sector_data.x_vec.push_back(mappnt_x_to_center);
					map_sector_1piece.map_sector_data.y_vec.push_back(mappnt_y_to_center);
					map_sector_1piece.map_sector_data.z_vec.push_back(0);
				}
			}
		}
		printf("sector cut finish. middle angle: %.2f, num of points in sector: %d\n", middle_angle * 180 / M_PI, map_sector_1piece.map_sector_data.x_vec.size());

	// }
}

bool CLS_GridMapLocalization::Check_Point_In_Sector(double angle_rad)
{
	if (angle_rad >= map_sector_1piece.start_angle && angle_rad <= map_sector_1piece.end_angle)
		return true;
	else if (angle_rad + 2 * M_PI >= map_sector_1piece.start_angle && angle_rad + 2 * M_PI <= map_sector_1piece.end_angle)
		return true;
	else if (angle_rad - 2 * M_PI >= map_sector_1piece.start_angle && angle_rad - 2 * M_PI <= map_sector_1piece.end_angle)
		return true;
	else
		return false;
}


/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: IcpMatch				   									█
█	Input: 																█
█	Output: 															█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
bool CLS_GridMapLocalization::pbl_fnc_IcpMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ent_cloud)
{
	// printf("HereA\n");
	pbl_STU_Pose2DStamped *my_matching_result_ = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;
	*my_matching_result_ = *pbl_Publish_Match;

	// cloud form to accelerate
	prv_STU_CloudMap my_Match_Cloud;

/*-------------------------------------------------*\
|	cloud form transformation  	    				|
\*-------------------------------------------------*/
#pragma region
	if (ent_cloud != nullptr)
	{
		unsigned int P_sum = ent_cloud->points.size();
		std::vector<float> x_vector(P_sum, 1.0f);
		std::vector<float> y_vector(P_sum, 1.0f);
		std::vector<float> z_vector(P_sum, 1.0f);
		for (unsigned int i = 0; i < P_sum; i++)
		{
			x_vector[i] = (ent_cloud->points[i].x);
			y_vector[i] = (ent_cloud->points[i].y);
			z_vector[i] = (ent_cloud->points[i].z);
		}
		my_Match_Cloud.x_vec.assign(x_vector.begin(), x_vector.end());
		my_Match_Cloud.y_vec.assign(y_vector.begin(), y_vector.end());
		my_Match_Cloud.z_vec.assign(z_vector.begin(), z_vector.end());
	}

	if (my_Match_Cloud.x_vec.empty() || pbl_GridMap2D.info.width == 0)
		return false;

#pragma endregion

	prv_STU_ICP_Params_Dynamic local_ICP_Params = prv_ICP_Params_Dynamic;

	bool keepApproaching;

	std::vector<prv_stu_MatchingPair> vec_MathingPair;
	pbl_STU_Pose2DStamped *my_lastMeanPose = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	unsigned short outInfo_nIterations = 0;

	// Asure maps are not empty!
	if (!my_Match_Cloud.x_vec.empty())
	{
		// ------------------------------------------------------
		//					The ICP loop
		// ------------------------------------------------------
		do
		{
			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			// printf("%d", local_ICP_Params.nPointOffset);
			my_determineMatching2D(my_Match_Cloud, my_matching_result_, vec_MathingPair, local_ICP_Params);

			if (!vec_MathingPair.size())
			{
				//  Nothing we can do !!
				keepApproaching = false;
			}
			else
			{
				//  Compute the estimated pose.
				//   (Method from paper of J.Gonzalez, Martinez y Morales)
				//  ----------------------------------------------------------------------
				keepApproaching = true;
				prv_se2_l2(vec_MathingPair, my_matching_result_);
				// n_vec_PointOffset.push_back(local_ICP_Params.nPointOffset);
				// If matching has not changed, decrease the thresholds:
				if (!(fabs(my_lastMeanPose->x - my_matching_result_->x) > prv_ICP_Params_Fixed.fMinAbsStep_trans ||
					  fabs(my_lastMeanPose->y - my_matching_result_->y) > prv_ICP_Params_Fixed.fMinAbsStep_trans ||
					  fabs(prv_fnc_wrapToPi(my_lastMeanPose->phi - my_matching_result_->phi)) > prv_ICP_Params_Fixed.fMinAbsStep_rot))
				{
					local_ICP_Params.fMaxDistForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
					local_ICP_Params.fMaxAnglForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
					if (local_ICP_Params.fMaxDistForCorres < prv_ICP_Params_Fixed.fMinThresholdDist)
						keepApproaching = false;

					if (++local_ICP_Params.nPointOffset >= prv_ICP_Params_Fixed.nPointDecimation)
						local_ICP_Params.nPointOffset = 0;
				}

				*my_lastMeanPose = *my_matching_result_;

			} // end of "else, there are correspondences"

			// Next iteration:
			outInfo_nIterations++;

			if (outInfo_nIterations >= prv_ICP_Params_Fixed.nMaxIterations &&
				local_ICP_Params.fMaxDistForCorres > prv_ICP_Params_Fixed.fMinThresholdDist)
			{
				local_ICP_Params.fMaxDistForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
			}
			// Yang test
			// vec_Yang_MatchingPair.push_back(vec_MathingPair);
			// pbl_vec_Yang_MatchResult.push_back(*my_matching_result_);
		} while ((keepApproaching && outInfo_nIterations < prv_ICP_Params_Fixed.nMaxIterations) ||
				 (outInfo_nIterations >= prv_ICP_Params_Fixed.nMaxIterations && local_ICP_Params.fMaxDistForCorres > prv_ICP_Params_Fixed.fMinThresholdDist));
	} // end of "if m2 is not empty"
	// nStep = prv_ICP_Params_Fixed.nPointDecimation;
	*pbl_Publish_Match = *my_matching_result_;
	printf("cloud size: %d, matching pair: %d.\n", ent_cloud->points.size(), vec_MathingPair.size());
	// pbl_MatchingPair = vec_MathingPair;
	// printf("ICP match finish.\n");
	// fprintf(fp_odometry, "%.6f\t%.6f\t%.6f\t%.6f\t\n", pbl_Publish_Match->timestamp, pbl_Publish_Match->x, pbl_Publish_Match->y, pbl_Publish_Match->phi);
	// printf("HereB\n");
	// printf("outInfo_nIterations: %d\n", outInfo_nIterations);
}


/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang, Xinao Wu	Create: 2023.10.10					█
█	Function: IcpMatch_Map2Map			   								█
█	Input: Map Vector													█
█	Output: 															█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
bool CLS_GridMapLocalization::pbl_fnc_IcpMatch_Map2Map(pbl_STU_Pose2DStamped *pose_disturbance)
{
	pbl_STU_Pose2DStamped *my_matching_result_ = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;
	*my_matching_result_ = *pbl_Publish_Match_Lasttime;
	// my_matching_result_->x = prv_map_center_x_ - 25 + pose_disturbance->x;
	// my_matching_result_->y = prv_map_center_y_ - 25 + pose_disturbance->y;
	my_matching_result_->phi = 0.0 + pose_disturbance->phi;

	pbl_MatchingPair.clear();

	// cloud form to accelerate
	prv_STU_CloudMap my_Match_Cloud;
/*-------------------------------------------------*\
|	cloud form transformation  	    				|
\*-------------------------------------------------*/
#pragma region
	if (!map_sector_1piece.map_sector_data.x_vec.empty() && pbl_GridMap2D.info.width != 0)
	{
		my_Match_Cloud.x_vec.assign(map_sector_1piece.map_sector_data.x_vec.begin(), map_sector_1piece.map_sector_data.x_vec.end());
		my_Match_Cloud.y_vec.assign(map_sector_1piece.map_sector_data.y_vec.begin(), map_sector_1piece.map_sector_data.y_vec.end());
		my_Match_Cloud.z_vec.assign(map_sector_1piece.map_sector_data.z_vec.begin(), map_sector_1piece.map_sector_data.z_vec.end());
	}
	else return false;

#pragma endregion

	prv_STU_ICP_Params_Dynamic local_ICP_Params = prv_ICP_Params_Dynamic;

	bool keepApproaching;

	std::vector<prv_stu_MatchingPair> vec_MathingPair;
	pbl_STU_Pose2DStamped *my_lastMeanPose = new CLS_GridMapLocalization::pbl_STU_Pose2DStamped;

	// The algorithm output auxiliar info:
	// -------------------------------------------------
	unsigned short outInfo_nIterations = 0;

	// Asure maps are not empty!
	if (!my_Match_Cloud.x_vec.empty())
	{
		// ------------------------------------------------------
		//					The ICP loop
		// ------------------------------------------------------
		do
		{
			// ------------------------------------------------------
			//		Find the matching (for a points map)
			// ------------------------------------------------------
			// printf("%d", local_ICP_Params.nPointOffset);
			my_determineMatching2D(my_Match_Cloud, my_matching_result_, vec_MathingPair, local_ICP_Params);

			if (!vec_MathingPair.size())
			{
				//  Nothing we can do !!
				keepApproaching = false;
			}
			else
			{
				//  Compute the estimated pose.
				//   (Method from paper of J.Gonzalez, Martinez y Morales)
				//  ----------------------------------------------------------------------
				keepApproaching = true;
				prv_se2_l2(vec_MathingPair, my_matching_result_);
				// n_vec_PointOffset.push_back(local_ICP_Params.nPointOffset);
				// If matching has not changed, decrease the thresholds:
				if (!(fabs(my_lastMeanPose->x - my_matching_result_->x) > prv_ICP_Params_Fixed.fMinAbsStep_trans ||
					  fabs(my_lastMeanPose->y - my_matching_result_->y) > prv_ICP_Params_Fixed.fMinAbsStep_trans ||
					  fabs(prv_fnc_wrapToPi(my_lastMeanPose->phi - my_matching_result_->phi)) > prv_ICP_Params_Fixed.fMinAbsStep_rot))
				{
					local_ICP_Params.fMaxDistForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
					local_ICP_Params.fMaxAnglForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
					if (local_ICP_Params.fMaxDistForCorres < prv_ICP_Params_Fixed.fMinThresholdDist)
						keepApproaching = false;

					if (++local_ICP_Params.nPointOffset >= prv_ICP_Params_Fixed.nPointDecimation)
						local_ICP_Params.nPointOffset = 0;
				}

				*my_lastMeanPose = *my_matching_result_;

			} // end of "else, there are correspondences"

			// Next iteration:
			outInfo_nIterations++;

			if (outInfo_nIterations >= prv_ICP_Params_Fixed.nMaxIterations &&
				local_ICP_Params.fMaxDistForCorres > prv_ICP_Params_Fixed.fMinThresholdDist)
			{
				local_ICP_Params.fMaxDistForCorres *= prv_ICP_Params_Fixed.fShrinkRatio;
			}
			// // Yang test
			// vec_Yang_MatchingPair.push_back(vec_MathingPair);
			// pbl_vec_Yang_MatchResult.push_back(*my_matching_result_);
		} while ((keepApproaching && outInfo_nIterations < prv_ICP_Params_Fixed.nMaxIterations) ||
				 (outInfo_nIterations >= prv_ICP_Params_Fixed.nMaxIterations && local_ICP_Params.fMaxDistForCorres > prv_ICP_Params_Fixed.fMinThresholdDist));
	} // end of "if m2 is not empty"
	// nStep = prv_ICP_Params_Fixed.nPointDecimation;
	pbl_Publish_Match_Assessment.push_back(*my_matching_result_);
	// pbl_Publish_Match_Assessment.push_back(*pbl_Publish_Match_Lasttime);
	pbl_MatchingPair.assign(vec_MathingPair.begin(), vec_MathingPair.end());
	printf("size of matching pair: %d\n", pbl_MatchingPair.size());

	//evolution 
	{
		float overlap = 100.0 * (float)vec_MathingPair.size() / my_Match_Cloud.x_vec.size();
		pbl_STU_Pose2DStamped pose_err;
		// pose_err.x = my_matching_result_->x - (prv_map_center_x_ - 25);
		// pose_err.y = my_matching_result_->y - (prv_map_center_y_ - 25);
		pose_err.x = my_matching_result_->x - pbl_Publish_Match_Lasttime->x;
		pose_err.y = my_matching_result_->y - pbl_Publish_Match_Lasttime->y;
		pose_err.phi = my_matching_result_->phi * 180 / M_PI;
		// printf("pose init: %.3f %.3f %.3fdeg\n", pose_disturbance->x, pose_disturbance->y, pose_disturbance->phi * 180 / M_PI);
		// printf("overlap: %.3f%, pose error: %.3f %.3f %.3fdeg\n", overlap, pose_err.x, pose_err.y, pose_err.phi);
		
		double score = pose_err.x * pose_err.x + pose_err.y * pose_err.y + pose_err.phi * pose_err.phi;
		scores.push_back(score);
		// fprintf(fp_odometry, "%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.6f\n", map_sector_1piece.middle_angle, overlap, pose_err.x, pose_err.y, sqrt(pose_err.x * pose_err.x + pose_err.y * pose_err.y), pose_err.phi, score);

	}
}


/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: my_determineMatching2D				   					█
█	Input: 																█
█	Output: 															█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
void CLS_GridMapLocalization::my_determineMatching2D(
	const prv_STU_CloudMap &ent_CloudMatch,
	const pbl_STU_Pose2DStamped *ent_Cloud_Pose,
	std::vector<prv_stu_MatchingPair> &exp_vec_MathingPair,
	const prv_STU_ICP_Params_Dynamic &params)
{
	const size_t nLocalPoints = ent_CloudMatch.x_vec.size();
	std::vector<float> x_locals(nLocalPoints), y_locals(nLocalPoints), z_locals(nLocalPoints);

	const float sin_phi = sin(ent_Cloud_Pose->phi);
	const float cos_phi = cos(ent_Cloud_Pose->phi);

	size_t nOtherMapPointsWithCorrespondence = 0; // Number of points with one corrs. at least
	size_t nTotalCorrespondences = 0;			  // Total number of corrs
	float _sumSqrDist = 0;

	// The number of cells to look around each point:
	const int cellsSearchRange = round(params.fMaxDistForCorres / pbl_GridMap2D.info.resolution); // 0.1 resolution

	exp_vec_MathingPair.clear();

	// Is local map empty?
	if (!nLocalPoints)
		return; // No

/*-------------------------------------------------*\
|	Only do matching if there is overlap			|
\*-------------------------------------------------*/
#pragma region

	float local_x_min = std::numeric_limits<float>::max();
	float local_x_max = -std::numeric_limits<float>::max();
	float local_y_min = std::numeric_limits<float>::max();
	float local_y_max = -std::numeric_limits<float>::max();
		
	//  Translate all local map points:
	for (unsigned int localIdx = params.nPointOffset; localIdx < nLocalPoints; localIdx += prv_ICP_Params_Fixed.nPointDecimation)
	{

		// Rotation and translation of the points
		const float xx = x_locals[localIdx] = ent_Cloud_Pose->x + cos_phi * ent_CloudMatch.x_vec[localIdx] - sin_phi * ent_CloudMatch.y_vec[localIdx];
		const float yy = y_locals[localIdx] = ent_Cloud_Pose->y + sin_phi * ent_CloudMatch.x_vec[localIdx] + cos_phi * ent_CloudMatch.y_vec[localIdx];
		z_locals[localIdx] = ent_CloudMatch.z_vec[localIdx];

		// mantener el max/min de los puntos:
		local_x_min = std::min(local_x_min, xx);
		local_x_max = std::max(local_x_max, xx);
		local_y_min = std::min(local_y_min, yy);
		local_y_max = std::max(local_y_max, yy);
	}

	// If the local map is entirely out of the grid,
	//   do not even try to match them!!
	if (local_x_min > pbl_GridMap2D.info.origin.position.x + pbl_GridMap2D.info.width * pbl_GridMap2D.info.resolution ||
		local_x_max < pbl_GridMap2D.info.origin.position.x ||
		local_y_min > pbl_GridMap2D.info.origin.position.y + pbl_GridMap2D.info.height * pbl_GridMap2D.info.resolution ||
		local_y_max < pbl_GridMap2D.info.origin.position.y)
		return; // Matching is NULL!

#pragma endregion

	// For each point in the other map:
	for (unsigned int localIdx = params.nPointOffset; localIdx < nLocalPoints; localIdx += prv_ICP_Params_Fixed.nPointDecimation)
	{
		// Starting value:
		float maxDistForCorrespondenceSquared = std::pow(params.fMaxDistForCorres, 2); // square( params.maxDistForCorrespondence );

		// For speed-up:
		const float x_local = x_locals[localIdx];
		const float y_local = y_locals[localIdx];
		const float z_local = z_locals[localIdx];
		prv_stu_MatchingPair my_closestCorr;

		// Look for the occupied cell closest from the map point:
		float min_dist = std::numeric_limits<float>::max(); // 1e6;

		// Will be set to true if a corrs. is found:
		bool thisLocalHasCorr = false;

		// Get the indexes of cell where the point falls:
		const int cx0 = static_cast<int>((x_local - pbl_GridMap2D.info.origin.position.x) / pbl_GridMap2D.info.resolution);
		const int cy0 = static_cast<int>((y_local - pbl_GridMap2D.info.origin.position.y) / pbl_GridMap2D.info.resolution);
		float x0 = pbl_GridMap2D.info.origin.position.x + (cx0 + 0.5f) * pbl_GridMap2D.info.resolution;
		float y0 = pbl_GridMap2D.info.origin.position.y + (cy0 + 0.5f) * pbl_GridMap2D.info.resolution;

		// Get the rectangle to look for into:
		const int cx_min = std::max(0, cx0 - cellsSearchRange);
		const int cx_max = std::min(static_cast<int>(pbl_GridMap2D.info.width) - 1, cx0 + cellsSearchRange);
		const int cy_min = std::max(0, cy0 - cellsSearchRange);
		const int cy_max = std::min(static_cast<int>(pbl_GridMap2D.info.height) - 1, cy0 + cellsSearchRange);

		//  Look in nearby cells:
		for (int cx = cx_min; cx <= cx_max; cx++)
		{
			for (int cy = cy_min; cy <= cy_max; cy++)
			{
				if (pbl_GridMap2D.data[cx + cy * pbl_GridMap2D.info.width] > 0)
				{
					const float residual_x = pbl_GridMap2D.info.origin.position.x + (cx + 0.5f) * pbl_GridMap2D.info.resolution - x_local;
					const float residual_y = pbl_GridMap2D.info.origin.position.y + (cy + 0.5f) * pbl_GridMap2D.info.resolution - y_local;

					// Compute max. allowed distance:
					maxDistForCorrespondenceSquared = std::pow(
						params.fMaxAnglForCorres *
								std::sqrt(std::pow(ent_Cloud_Pose->x - x_local, 2) + std::pow(ent_Cloud_Pose->y - y_local, 2)) +
							params.fMaxDistForCorres,
						2);

					// Square distance to the point:
					const float this_dist = std::pow(residual_x, 2) + std::pow(residual_y, 2);
					if (this_dist < maxDistForCorrespondenceSquared)
					{
						// At least one:
						thisLocalHasCorr = true;

						// save the closest only:
						if (this_dist < min_dist)
						{
							// ("%f\n", this_dist);
							min_dist = this_dist;

							my_closestCorr.Map_idx = cx + cy * pbl_GridMap2D.info.width;											   // pbl_GridMap2D->size_x;
							my_closestCorr.Map_x = pbl_GridMap2D.info.origin.position.x + (cx + 0.5f) * pbl_GridMap2D.info.resolution; // pbl_GridMap2D->idx2x(cx);
							my_closestCorr.Map_y = pbl_GridMap2D.info.origin.position.y + (cy + 0.5f) * pbl_GridMap2D.info.resolution; // pbl_GridMap2D->idx2y(cy);
							my_closestCorr.Map_z = z_local;
							my_closestCorr.Cld_idx = localIdx;
							my_closestCorr.Cld_x = ent_CloudMatch.x_vec[localIdx];
							my_closestCorr.Cld_y = ent_CloudMatch.y_vec[localIdx];
							my_closestCorr.Cld_z = ent_CloudMatch.z_vec[localIdx];
						}
					}
				}
			}
		} // End of find closest nearby cell

		// save the closest correspondence:
		if (min_dist < maxDistForCorrespondenceSquared)
		{
			nTotalCorrespondences++;
			exp_vec_MathingPair.push_back(my_closestCorr);
		}

		// At least one corr:
		if (thisLocalHasCorr)
		{
			nOtherMapPointsWithCorrespondence++;
			_sumSqrDist += min_dist;
		}
		// printf("min_dist = %f\n", min_dist);
	} // End "for each local point"...
	float extraResults_correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints / prv_ICP_Params_Fixed.nPointDecimation);
	float extraResults_sumSqrDist = _sumSqrDist;
}

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Author: Chenxi Yang		Create: 2021.11.09							█
█	Function: prv_se2_l2							   					█
█	Input: 																█
█	Output: 															█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
bool CLS_GridMapLocalization::prv_se2_l2(std::vector<prv_stu_MatchingPair> &prv_vec_MathingPair, pbl_STU_Pose2DStamped *out_transformation)
{
	const size_t N = prv_vec_MathingPair.size();
	if (N < 2)
		return false;

	const float N_inv = 1.0f / N; // For efficiency, keep this value.

	// ----------------------------------------------------------------------
	// Compute the estimated pose. Notation from the paper:
	// "Mobile robot motion estimation by 2d scan matching with genetic and iterative
	// closest point algorithms", J.L. Martinez Rodriguez, A.J. Gonzalez, J. Morales
	// Rodriguez, A. Mandow Andaluz, A. J. Garcia Cerezo,
	// Journal of Field Robotics, 2006.
	// ----------------------------------------------------------------------

	// ----------------------------------------------------------------------
	//  For the formulas of the covariance, see:
	//   http://www.mrpt.org/Paper:Occupancy_Grid_Matching
	//   and Jose Luis Blanco's PhD thesis.
	// ----------------------------------------------------------------------

	//#if MRPT_HAS_SSE2
	__m128 sum_a_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)
	__m128 sum_b_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)

	//   [ f0     f1      f2      f3  ]
	//    xa*xb  ya*yb   xa*yb  xb*ya
	__m128 sum_ab_xyz = _mm_setzero_ps(); // All 4 zeros (0.0f)

	for (std::vector<prv_stu_MatchingPair>::const_iterator corrIt = prv_vec_MathingPair.begin(); corrIt != prv_vec_MathingPair.end(); corrIt++)
	{
		// Get the pair of points in the correspondence:
		//   a_xyyx = [   xa     ay   |   xa    ya ]
		//   b_xyyx = [   xb     yb   |   yb    xb ]
		//      (product)
		//            [  xa*xb  ya*yb   xa*yb  xb*ya
		//                LO0    LO1     HI2    HI3
		// Note: _MM_SHUFFLE(hi3,hi2,lo1,lo0)
		const __m128 a_xyz = _mm_loadu_ps(&corrIt->Map_x); // *Unaligned* load
		const __m128 b_xyz = _mm_loadu_ps(&corrIt->Cld_x); // *Unaligned* load

		const __m128 a_xyxy = _mm_shuffle_ps(a_xyz, a_xyz, _MM_SHUFFLE(1, 0, 1, 0));
		const __m128 b_xyyx = _mm_shuffle_ps(b_xyz, b_xyz, _MM_SHUFFLE(0, 1, 1, 0));

		// Compute the terms:
		sum_a_xyz = _mm_add_ps(sum_a_xyz, a_xyz);
		sum_b_xyz = _mm_add_ps(sum_b_xyz, b_xyz);

		//   [ f0     f1      f2      f3  ]
		//    xa*xb  ya*yb   xa*yb  xb*ya
		sum_ab_xyz = _mm_add_ps(sum_ab_xyz, _mm_mul_ps(a_xyxy, b_xyyx));
	}

	__attribute__((aligned(16))) float sums_a[4], sums_b[4];
	_mm_store_ps(sums_a, sum_a_xyz);
	_mm_store_ps(sums_b, sum_b_xyz);

	const float &SumXa = sums_a[0];
	const float &SumYa = sums_a[1];
	const float &SumXb = sums_b[0];
	const float &SumYb = sums_b[1];

	// Compute all four means:
	const __m128 Ninv_4val = _mm_set1_ps(N_inv); // load 4 copies of the same value
	sum_a_xyz = _mm_mul_ps(sum_a_xyz, Ninv_4val);
	sum_b_xyz = _mm_mul_ps(sum_b_xyz, Ninv_4val);

	// means_a[0]: mean_x_a
	// means_a[1]: mean_y_a
	// means_b[0]: mean_x_b
	// means_b[1]: mean_y_b
	__attribute__((aligned(16))) float means_a[4], means_b[4];
	_mm_store_ps(means_a, sum_a_xyz);
	_mm_store_ps(means_b, sum_b_xyz);

	const float &mean_x_a = means_a[0];
	const float &mean_y_a = means_a[1];
	const float &mean_x_b = means_b[0];
	const float &mean_y_b = means_b[1];

	//      Sxx   Syy     Sxy    Syx
	//    xa*xb  ya*yb   xa*yb  xb*ya
	__attribute__((aligned(16))) float cross_sums[4];
	_mm_store_ps(cross_sums, sum_ab_xyz);

	const float &Sxx = cross_sums[0];
	const float &Syy = cross_sums[1];
	const float &Sxy = cross_sums[2];
	const float &Syx = cross_sums[3];

	// Auxiliary variables Ax,Ay:
	const float Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
	const float Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;
	//#endif

	out_transformation->phi = (Ax != 0 || Ay != 0) ? atan2(static_cast<double>(Ay), static_cast<double>(Ax)) : 0.0;

	const double ccos = cos(out_transformation->phi);
	const double csin = sin(out_transformation->phi);

	out_transformation->x = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
	out_transformation->y = mean_y_a - mean_x_b * csin - mean_y_b * ccos;
	return true;
}