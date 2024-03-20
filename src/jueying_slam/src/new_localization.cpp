

#include "utility.h"
#include "jueying_slam/cloud_info.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/passthrough.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include "dynamic_map.h"
#include "tictoc.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

using namespace gtsam;

using symbol_shorthand::X; 
using symbol_shorthand::V; 
using symbol_shorthand::B; 
using symbol_shorthand::G; 

class NewLocalization : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;

    const int *queueLength =new int(imuFrequency);

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate; 

    ISAM2 *isam;  
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance; 

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    double *imuTime = new double[*queueLength];
    double *imuRotX = new double[*queueLength];
    double *imuRotY = new double[*queueLength];
    double *imuRotZ = new double[*queueLength];

    int imuPointerCur; 
    int OdomPointerCur; 
    bool firstPointFlag; 
    Eigen::Affine3f transStartInverse;  

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D; 
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn; 

    pcl::PointCloud<PointType>::Ptr   fullCloud;    
    pcl::PointCloud<PointType>::Ptr   extractedCloud; 

    int deskewFlag; 
    cv::Mat rangeMat; 

    bool odomDeskewFlag; 
    double *odomTime = new double[*queueLength];
    double *odomIncreX = new double[*queueLength];
    double *odomIncreY = new double[*queueLength];
    double *odomIncreZ = new double[*queueLength];

    jueying_slam::cloud_info cloudInfo;
    double timeScanCur; 

    std_msgs::Header cloudHeader;

    pcl::PointCloud<PointType>::Ptr cornerCloud;  
    pcl::PointCloud<PointType>::Ptr surfaceCloud; 

    pcl::VoxelGrid<PointType> downSizeFilter;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;  

    pcl::PointCloud<PointType>::Ptr globalMap;
    bool initial=false;  
    bool load_map=false;  
    pcl::Registration<PointType, PointType>::Ptr registration; 

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;

     ros::Publisher pubPath;

    ros::Publisher pubCloudRegisteredRaw;

    ros::Subscriber subGPS;   
    ros::Subscriber subUKF;

    std::deque<nav_msgs::Odometry> gpsQueue; 

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; 
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; 
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; 
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; 

    pcl::PointCloud<PointType>::Ptr laserCloudOri; 
    pcl::PointCloud<PointType>::Ptr coeffSel;   

    std::vector<PointType> laserCloudOriCornerVec; 
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; 
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap; 
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS; 
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::VoxelGrid<PointType> downSizeFilterMap;
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;

    pcl::PassThrough<PointType> pass;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6]; 
    float last_loadMap[6];
    std::mutex mtx;
    std::mutex Mapmtx;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;
    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront; 
    Eigen::Affine3f incrementalOdometryAffineBack; 

    double Corner_fitness_score,Surf_fitness_score;
    int Corner_num=0,Surf_num=0;
    bool gps_initailized;
    bool map_initailized;
    bool pose_initailized;
    int initial_count=0;
    bool lose_flag;
    bool ukf_initialized=false;

public:
    NewLocalization():deskewFlag(0),gps_initailized(false),map_initailized(false),pose_initailized(false),lose_flag(false)
    {

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &NewLocalization::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &NewLocalization::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &NewLocalization::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("jueying_slam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("jueying_slam/mapping/odometry", 1); 
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("jueying_slam/mapping/odometry_incremental", 1); 
        pubPath                     = nh.advertise<nav_msgs::Path>("jueying_slam/mapping/path", 1); 

        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &NewLocalization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subUKF   = nh.subscribe<nav_msgs::Odometry> ("ukf/odom", 1, &NewLocalization::UKFHandler, this, ros::TransportHints().tcpNoDelay());
        allocateMemory();
        initializationValue();
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize*1.5, mappingSurfLeafSize*1.5, mappingSurfLeafSize*1.5);
         if(mintialMethod==human)
            gps_initailized=true;

    }

    void initializationValue()
    {

        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    void allocateMemory()
    {   

        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); 
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); 
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); 
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); 

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMap->header.frame_id = "map";
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap->header.frame_id = "map";
        globalMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        all_Corner_areas = read_arealist(globalCornerMap_pcd);
        all_Surf_areas = read_arealist(globalSurfMap_pcd);

        if (margin < 0)  
        {
            for(const Area& area : all_Corner_areas){
            Corner_pcd_file_paths.push_back(globalCornerMap_dirctory+area.path);
            }
            for(const Area& area : all_Surf_areas){
                Surf_pcd_file_paths.push_back(globalSurfMap_dirctory+area.path);
            }
            laserCloudCornerFromMap=create_pcd(Corner_pcd_file_paths);
            laserCloudSurfFromMap=create_pcd(Surf_pcd_file_paths);

            load_map = true;

        }

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
            last_loadMap[i]=-999999;
        }

        matP.setZero();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        OdomPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < *queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            odomTime[i] = 0;
            odomIncreX[i] = 0;
            odomIncreY[i] = 0;
            odomIncreZ[i] = 0;
        }
    }

    ~NewLocalization(){delete queueLength;}

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {

        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); 
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }
    void dynamic_load_map(const float pose[]){
        if (margin >= 0){

            pass.setInputCloud (laserCloudCornerFromMap);
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (pose[3]-max_range*1.1, pose[3]+max_range*1.1);
            pass.filter (*laserCloudCornerFromMapDS);

            pass.setInputCloud (laserCloudSurfFromMap);
            pass.filter (*laserCloudSurfFromMapDS);

            pass.setInputCloud (laserCloudCornerFromMap);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (pose[4]-max_range*1.1, pose[4]+max_range*1.1);
            pass.filter (*laserCloudCornerFromMapDS);

            pass.setInputCloud (laserCloudSurfFromMap);
            pass.filter (*laserCloudSurfFromMapDS);
            *globalMap =*laserCloudCornerFromMapDS+ *laserCloudSurfFromMapDS;

            if(Matching_method=="ndt")
                registration->setInputTarget(globalMap);

        }
    }

    void dynamic_load_map_run(){
        ros::Rate rate(updateMapFrequency);
        while (ros::ok())
        {
            if(!gps_initailized)
                continue;
            if(!map_initailized){

                transformTobeMapped[3] = initialPose[0];
                transformTobeMapped[4] = initialPose[1];
                transformTobeMapped[5] = initialPose[2];
                // cout<<"initial pose is"<<initialPose[0]<<","<<initialPose[1]<<","<<initialPose[2]<<endl;  
            }

            float distance_x = transformTobeMapped[3]-last_loadMap[3];
            float distance_y = transformTobeMapped[4]-last_loadMap[4];
            float distance_z = transformTobeMapped[5]-last_loadMap[5];
            float load_distance=sqrt(distance_x*distance_x+distance_y*distance_y+distance_z*distance_z);

            if(load_distance>area_size){
                std::lock_guard<std::mutex> lock(Mapmtx);
                std::cout<<"加载地图"<<std::endl;
                laserCloudCornerFromMap=create_pcd(transformTobeMapped[3],transformTobeMapped[4],all_Corner_areas,globalCornerMap_dirctory,margin);
                laserCloudSurfFromMap=create_pcd(transformTobeMapped[3],transformTobeMapped[4],all_Surf_areas,globalSurfMap_dirctory,margin);
                dynamic_load_map(transformTobeMapped);
                publishCloud(&pubLaserCloudSurround, globalMap, ros::Time::now(), "map");
                map_initailized=true;
                for (int i = 3; i < 6; ++i){
                    last_loadMap[i]=transformTobeMapped[i] ;
                }
                load_map=true;
            }
            rate.sleep();   
        } 
    }

    double my_getFitnessScore(pcl::PointCloud<PointType>::Ptr input_cloud,double max_range)
    {

        updatePointAssociateToMap();

        double fitness_score = 0.0;

        int nr = 0;
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < input_cloud->size(); ++i)
        {
            PointType pointOri, pointSel;
            std::vector<int> nn_indices;
            std::vector<float> nn_dists;
            pointOri = input_cloud->points[i];
            pointAssociateToMap(&pointOri, &pointSel);

            kdtreeCornerFromMap->nearestKSearch (pointSel, 1, nn_indices, nn_dists);

            if (nn_dists[0] <= max_range)
            {

                fitness_score += nn_dists[0];
                nr++;
            }
        }

        if (nr > input_cloud->points.size ()*0.3)
            return (fitness_score / nr);
        else
            return (std::numeric_limits<double>::max ());

    }

    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {

            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            cloudCurvature[i] = diffRange*diffRange;

            cloudNeighborPicked[i] = 0; 
            cloudLabel[i] = 0; 

            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();

        for (int i = 5; i < cloudSize - 6; ++i)
        {

            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];

            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            if (columnDiff < 10){ 

                if (depth1 - depth2 > 0.3){ 
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;

        }
    }

    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            for (int j = 0; j < area_num; j++)
            {

                int sp = (cloudInfo.startRingIndex[i] * (area_num - j) + cloudInfo.endRingIndex[i] * j) / area_num;
                int ep = (cloudInfo.startRingIndex[i] * (area_num - 1 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / area_num - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                int largestPickedNum = 0; 
                for (int k = ep; k >= sp; k--)
                {

                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0   
                        && cloudCurvature[ind] > edgeThreshold) 
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20){ 
                            cloudLabel[ind] = 1; 
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; 

                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){ 
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {

        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    int map_count=0;
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        TicToc t1(true);

        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        cloudInfo.header = laserCloudMsg->header;

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        if(!gps_initailized){
            cout<<"GPS not initailized"<<endl;
            return;
        }
        if(!map_initailized){
            cout<<"Map not initailized"<<endl;
            return;
        }

        timeLaserInfoStamp = laserCloudMsg->header.stamp;
        timeLaserInfoCur = laserCloudMsg->header.stamp.toSec();

        laserCloudCornerLast=cornerCloud;
        laserCloudSurfLast=surfaceCloud;
        cout<<"laserCloudCornerLast"<<laserCloudCornerLast->size()<<endl;

        std::lock_guard<std::mutex> lock(Mapmtx);

        updateInitialGuess();
        if(load_map==false)
            return;

        downsampleCurrentScan();

        dynamic_load_map(transformTobeMapped);

        if(map_count<10){
            map_count++;
               return;
        }

        scan2MapOptimization();

        std::cout<<"焦点匹配分数是"<<Corner_fitness_score<<std::endl;
        std::cout<<"面点匹配分数是"<<Surf_fitness_score<<std::endl;

        initial_count++;
        if(Corner_fitness_score<0.3&&initial_count>initial_count_num){

            initial_count=initial_count_num;
            updatePath(transformTobeMapped);
            publishOdometry();
            publishFrames();
            lose_flag=false; 
        }
        else if(initial_count<initial_count_num+1){

            updatePath(transformTobeMapped);
            publishOdometry();
            publishFrames();

        }

        else{

            cout<<"定位失败重定位"<<endl;

            if(ukf_initialized)
            {

                transformTobeMapped[0] = cloudInfo.imuRollInit;
                transformTobeMapped[1] = cloudInfo.imuPitchInit;
                transformTobeMapped[2] = cloudInfo.imuYawInit;

                transformTobeMapped[3] = initialPose[0];
                transformTobeMapped[4] = initialPose[1];
                transformTobeMapped[5] = initialPose[2];

                scan2MapOptimization();

                std::cout<<"重定位焦点匹配分数是"<<Corner_fitness_score<<std::endl;
                std::cout<<"重定位面点匹配分数是"<<Surf_fitness_score<<std::endl;
                if(Corner_fitness_score<0.1){

                }
                else{
                    transformTobeMapped[0] = cloudInfo.imuRollInit;
                    transformTobeMapped[1] = cloudInfo.imuPitchInit;
                    transformTobeMapped[2] = cloudInfo.imuYawInit;

                    transformTobeMapped[3] = initialPose[0];
                    transformTobeMapped[4] = initialPose[1];
                    transformTobeMapped[5] = initialPose[2];
                    lose_flag=true;

                }
            }

            updatePath(transformTobeMapped);
            publishOdometry();

            publishFrames();

        }

        t1.toc("运行用时");

        resetParameters();
        freeCloudInfoMemory();
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        addOdomFactor();

        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); 
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; 
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

    }

    void updatePath(const float pose[])
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = transformTobeMapped[3];
        pose_stamped.pose.position.y = transformTobeMapped[4];
        pose_stamped.pose.position.z = 0;
        tf::Quaternion q = tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void UKFHandler(const nav_msgs::Odometry::ConstPtr& ukfMsg){

        if(!ukf_initialized)
            ukf_initialized=true;
        Eigen::Vector3d Pwl;
        Eigen::Vector3d Pwi(ukfMsg->pose.pose.position.x,ukfMsg->pose.pose.position.y,ukfMsg->pose.pose.position.z);
        Eigen::Quaterniond Qwi(ukfMsg->pose.pose.orientation.w,ukfMsg->pose.pose.orientation.x,ukfMsg->pose.pose.orientation.y,ukfMsg->pose.pose.orientation.z);
        Pwl= Pwi+ Qwi.matrix()*Pil;
        initialPose.at(0)=Pwl.x();
        initialPose.at(1)=Pwl.y();
        initialPose.at(2)=Pwl.z();

    }

    int gps_count=0;
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last;
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

         if(mintialMethod==gps){
            if(!gps_initailized&&(gpsMsg->pose.pose.position.x!=0||gpsMsg->pose.pose.position.y!=0)&&(gpsMsg->pose.covariance[0]<0.003&&gpsMsg->pose.covariance[7]<0.003)){

                Eigen::Vector3d Pwl;
                Eigen::Vector3d Pwi(gpsMsg->pose.pose.position.x,gpsMsg->pose.pose.position.y,gpsMsg->pose.pose.position.z);
                Eigen::Quaterniond Qwi(gpsMsg->pose.pose.orientation.w,gpsMsg->pose.pose.orientation.x,gpsMsg->pose.pose.orientation.y,gpsMsg->pose.pose.orientation.z);
                Pwl= Pwi+ Qwi.matrix()*Pil;
                cout<<"GPS initailizes"<<endl;
                initialPose.at(0)=Pwl.x();
                initialPose.at(1)=Pwl.y();

                gps_initailized=true;

            }
        }

    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    void updateInitialGuess()
    {

        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation; 

        if(!pose_initailized||initial_count<initial_count_num||lose_flag)
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            transformTobeMapped[3] = initialPose[0];
            transformTobeMapped[4] = initialPose[1];
            transformTobeMapped[5] = initialPose[2];

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0,0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
            pose_initailized=true;
            return;
        }

        static bool lastImuPreTransAvailable = false; 
        static Eigen::Affine3f lastImuPreTransformation; 
        if (cloudInfo.odomAvailable == true)
        {

            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {

                lastImuPreTransformation = transBack;

                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped); 
                Eigen::Affine3f transFinal = transTobe * transIncre; 
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack; 

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
                return;
            }
        }

        if (cloudInfo.imuAvailable == true)
        {

            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);

            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); 
            return;
        }
    }

    void downsampleCurrentScan()
    {

        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[0] <= 1.0)
            {

                Corner_fitness_score += pointSearchSqDis[0];
                Corner_num++;
            }

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0) {

                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;

                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];

            pointAssociateToMap(&pointOri, &pointSel); 
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[0] <= 1.0)
            {

                Surf_fitness_score += pointSearchSqDis[0];
                Surf_num++;
            }
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                            + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {

        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }

        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {

        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);  
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {

            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;

            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;

            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;

            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;

            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;

        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; 
        }
        return false; 
    }

    void scan2MapOptimization()
    {

        if(!pose_initailized)
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {

            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < iter_num; iterCount++)
            {
                Corner_fitness_score=0.0;
                Surf_fitness_score=0.0;
                Corner_num=0;Surf_num=0;

                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                if (Corner_num > 1)
                    Corner_fitness_score= (Corner_fitness_score / (double)Corner_num);
                else 
                    Corner_fitness_score= (std::numeric_limits<double>::max ());
                surfOptimization();
                if (Surf_num > 1)
                    Surf_fitness_score= (Surf_fitness_score / (double)Surf_num);
                else 
                    Surf_fitness_score= (std::numeric_limits<double>::max ());

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    void publishOdometry()
    {

        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometryROS.pose.covariance[0]=Corner_fitness_score;
        laserOdometryROS.pose.covariance[1]=Surf_fitness_score;
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, lidarFrame);
        br.sendTransform(trans_odom_to_lidar);

        pubLaserOdometryIncremental.publish(laserOdometryROS);
    }

    void publishFrames()
    {

        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        publishCloud(&pubLaserCloudSurround, globalMap, ros::Time::now(), "map");
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }
    void myPassThrough (const pcl::PointCloud<PointXYZIRT> &cloud_in, 
                              pcl::PointCloud<PointXYZIRT> &cloud_out,
                        float limit)
    {

        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize (cloud_in.points.size ());
        }
        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size (); ++i)
        {
            if ((cloud_in.points[i].x*cloud_in.points[i].x+cloud_in.points[i].y*cloud_in.points[i].y)>limit)              
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size ())
        {

            cloud_out.points.resize (j);
        }

        cloud_out.height = cloud_in.height;
        cloud_out.width  = static_cast<uint32_t>(j)/cloud_in.height; 
    }

    void myremoveNaNFromPointCloud (const pcl::PointCloud<PointXYZIRT> &cloud_in, 
                              pcl::PointCloud<PointXYZIRT> &cloud_out,
                              std::vector<int> &index)
    {

        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize (cloud_in.points.size ());
        }

        index.resize (cloud_in.points.size ());
        size_t j = 0;

        if (cloud_in.is_dense)
        {

            cloud_out = cloud_in;
            for (j = 0; j < cloud_out.points.size (); ++j)
            index[j] = static_cast<int>(j);
        }
        else
        {
            for (size_t i = 0; i < cloud_in.points.size (); ++i)
            {
            if (!pcl_isfinite (cloud_in.points[i].x) || 
                !pcl_isfinite (cloud_in.points[i].y) || 
                !pcl_isfinite (cloud_in.points[i].z))
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            index[j] = static_cast<int>(i);
            j++;
            }
            if (j != cloud_in.points.size ())
            {

            cloud_out.points.resize (j);
            index.resize (j);
            }

            cloud_out.height = cloud_in.height;
            cloud_out.width  = static_cast<uint32_t>(j)/cloud_in.height;

            cloud_out.is_dense = true;
        }
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {

        timeScanCur = laserCloudMsg->header.stamp.toSec(); 

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        if(mlidar_type==Velodyne)
        {

            if (laserCloudIn->is_dense == false)
            {
                std::cout<< "laserCloudIn->is_dense"<<laserCloudIn->is_dense<<std::endl;
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }

            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)laserCloudMsg->fields.size(); ++i)
                {
                    if (laserCloudMsg->fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                if (ringFlag == -1)
                {
                    ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                    ros::shutdown();
                }
            }   

            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)laserCloudMsg->fields.size(); ++i)
                {
                    if (laserCloudMsg->fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                if (deskewFlag == -1)
                    ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }
        }

        else if(mlidar_type==rslidar_16)
        {   

            #pragma omp parallel for num_threads(numberOfCores)
            for(int h=0;h<laserCloudIn->height-1;++h){
                for(int w=0;w<laserCloudIn->width-1;++w){
                    laserCloudIn->at(w,h).ring=h<8?h:15-(h-8);
                }
            } 

             if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)laserCloudMsg->fields.size(); ++i)
                {
                    if (laserCloudMsg->fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }

                if(deskewFlag!=1)
                {
                    #pragma omp parallel for num_threads(numberOfCores)
                    for(int h=0;h<laserCloudIn->height-1;++h){
                        for(int w=0;w<laserCloudIn->width-1;++w){
                            laserCloudIn->at(w,h).time=h*2.8*1e-6+w*55.5*1e-6;
                        }
                    } 
                    deskewFlag=1;
                }
            }

            if (laserCloudIn->is_dense == false)
            {

                std::vector<int> indices;
                myremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            }
        }

        else if(mlidar_type==rslidar_ruby){

            #pragma omp parallel for num_threads(numberOfCores)
            for(int h=0;h<laserCloudIn->height-1;++h){
                for(int w=0;w<laserCloudIn->width-1;++w){
                    laserCloudIn->at(w,h).ring=h;
                }
            } 

            if (deskewFlag == 0) {
                deskewFlag = -1;
                for (int i = 0; i < (int) laserCloudMsg->fields.size(); ++i) {
                    if (laserCloudMsg->fields[i].name == timeField) {
                        deskewFlag = 1;
                        break;
                    }
                }

                if (deskewFlag != 1) {
                    #pragma omp parallel for num_threads(numberOfCores)
                    for (int h = 0; h < laserCloudIn->height - 1; ++h) {
                        for (int w = 0; w < laserCloudIn->width - 1; ++w) {
                            laserCloudIn->at(w, h).time = 3.236*((((h+1)%64==0?64:(h+1)%64)-1)/4.f) * 1e-6 + w * 55.552 * 1e-6;
                        }
                    }
                    deskewFlag = 1;
                }
            }

            if (laserCloudIn->is_dense == false) {

                std::vector<int> indices;
                myremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            }
        }

        else if(mlidar_type==rslidar_32){

            #pragma omp parallel for num_threads(numberOfCores)
            for(int h=0;h<laserCloudIn->height-1;++h){
                for(int w=0;w<laserCloudIn->width-1;++w){
                    laserCloudIn->at(w,h).ring=h;
                }
            } 

            if (deskewFlag == 0) {
                deskewFlag = -1;
                for (int i = 0; i < (int) laserCloudMsg->fields.size(); ++i) {
                    if (laserCloudMsg->fields[i].name == timeField) {
                        deskewFlag = 1;
                        break;
                    }
                }

                if (deskewFlag != 1) {
                    #pragma omp parallel for num_threads(numberOfCores)
                    for (int h = 0; h < laserCloudIn->height - 1; ++h) {
                        for (int w = 0; w < laserCloudIn->width - 1; ++w) {
                            laserCloudIn->at(w, h).time = (w*55.52+h%16*2.88+1.44*float(h/16))*1e-6;
                        }
                    }
                    deskewFlag = 1;
                }
            }

            if (laserCloudIn->is_dense == false) {

                std::vector<int> indices;
                myremoveNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
            }
        }
        else{
            ROS_ERROR("Undefined lidar type");
            exit(-1);
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur )
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;
        double imuRotX_0,imuRotY_0,imuRotZ_0;
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (currentImuTime <= timeScanCur)

                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (imuPointerCur == 0){
                imuRPY2rosRPY(&thisImuMsg, &imuRotX_0, &imuRotY_0, &imuRotZ_0);
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;  
                continue;
            }

            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;

            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0) 
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        odomDeskewFlag = false;
        OdomPointerCur = 0;

        double odomIncreX_0,odomIncreY_0,odomIncreZ_0;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            nav_msgs::Odometry thisOdomMsg = odomQueue[i];
            double currentOdomTime = thisOdomMsg.header.stamp.toSec();

            if (OdomPointerCur == 0){
                odomIncreX_0 = thisOdomMsg.pose.pose.position.x;
                odomIncreY_0 = thisOdomMsg.pose.pose.position.y;
                odomIncreZ_0 = thisOdomMsg.pose.pose.position.z;
                odomIncreX[0] = 0;
                odomIncreY[0] = 0;
                odomIncreZ[0] = 0;
                odomTime[0] = currentOdomTime;
                ++OdomPointerCur; 
                continue;
            }

            odomIncreX[OdomPointerCur] = thisOdomMsg.pose.pose.position.x - odomIncreX_0;
            odomIncreY[OdomPointerCur] = thisOdomMsg.pose.pose.position.y - odomIncreY_0;
            odomIncreZ[OdomPointerCur] = thisOdomMsg.pose.pose.position.z - odomIncreZ_0;
            odomTime[OdomPointerCur] = currentOdomTime;
            ++OdomPointerCur;
        }

        --OdomPointerCur;

        if (OdomPointerCur <= 0) 
            return;

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;

        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else { 

            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }

    }

    void findPosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        int OdomPointerFront = 0;

        while (OdomPointerFront < OdomPointerCur)
        {
            if (pointTime < odomTime[OdomPointerFront])
                break;
            ++OdomPointerFront;
        }

        if (pointTime > odomTime[OdomPointerFront] || OdomPointerFront == 0)
        {
            *posXCur = odomIncreX[OdomPointerFront];
            *posYCur = odomIncreY[OdomPointerFront];
            *posZCur = odomIncreZ[OdomPointerFront];
        } else { 

            int OdomPointerBack = OdomPointerFront - 1;
            double ratioFront = (pointTime - odomTime[OdomPointerBack]) / (odomTime[OdomPointerFront] - odomTime[OdomPointerBack]);
            double ratioBack = (odomTime[OdomPointerFront] - pointTime) / (odomTime[OdomPointerFront] - odomTime[OdomPointerBack]);
            *posXCur = odomIncreX[OdomPointerFront] * ratioFront + odomIncreX[OdomPointerBack] * ratioBack;
            *posYCur = odomIncreY[OdomPointerFront] * ratioFront + odomIncreY[OdomPointerBack] * ratioBack;
            *posZCur = odomIncreZ[OdomPointerFront] * ratioFront + odomIncreZ[OdomPointerBack] * ratioBack;
        }

    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime; 

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int rowIdn;

        static float ang_res_x = 360.0/float(Horizon_SCAN);

        int cloudSize = laserCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0) 
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            float range = pointDistance(thisPoint);

            if (range < min_range || range >max_range)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = pointDistance(thisPoint);

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }

        for (int i = 0; i < *queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            odomTime[i] = 0;
            odomIncreX[i] = 0;
            odomIncreY[i] = 0;
            odomIncreZ[i] = 0;
        }

    }

    void cloudExtraction()
    {
        int count = 0;

        for (int i = 0; i < N_SCAN; ++i) 
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)  
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {

                    cloudInfo.pointColInd[count] = j;

                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);

                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);

                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jueying_slam");

    NewLocalization NL;

    ROS_INFO("\033[1;32m----> new localization Started.\033[0m");

    std::thread Dynamic_loadMap(&NewLocalization::dynamic_load_map_run,&NL);

    ros::MultiThreadedSpinner spinner(6);
    spinner.spin();

    Dynamic_loadMap.join();

    return 0;
}