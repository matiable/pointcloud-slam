

#include "utility.h"
#include "jueying_slam/cloud_info.h"
#include <pcl/filters/passthrough.h>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (uint8_t, intensity, intensity)
    (double, timestamp, timestamp) (uint16_t, ring, ring) 
)

struct PointXYZIRT_RS
{
    float x;
    float y;
    float z;
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
};

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;

    const int *queueLength =new int(imuFrequency);

    ros::Publisher pubExtractedCloud;

    ros::Publisher pubLaserCloudInfo;

    ros::Publisher pubPointsOneRing;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[*queueLength];
    double *imuRotX = new double[*queueLength];
    double *imuRotY = new double[*queueLength];
    double *imuRotZ = new double[*queueLength];

    int imuPointerCur;
    int OdomPointerCur;
    bool firstPointFlag; 
    Eigen::Affine3f transStartInverse; 

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
    double timeScanNext;
    std_msgs::Header cloudHeader;

public:
    ImageProjection():deskewFlag(0)
    {

        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("jueying_slam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<jueying_slam::cloud_info> ("jueying_slam/deskew/cloud_info", 1);
        pubPointsOneRing = nh.advertise<sensor_msgs::PointCloud2> ("rslidar_32_one_ring",1);

        allocateMemory();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
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

    ~ImageProjection(){delete queueLength;}

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

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
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

        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;
        else
        {
            currentCloudMsg = cloudQueue.front();
            cloudQueue.pop_front();

            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec(); 
            timeScanNext = cloudQueue.front().header.stamp.toSec();
        }

        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        
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
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
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
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
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
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
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
                            laserCloudIn->at(w,h).timestamp=h*2.8*1e-6+w*55.5*1e-6;
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
                for (int i = 0; i < (int) currentCloudMsg.fields.size(); ++i) {
                    if (currentCloudMsg.fields[i].name == timeField) {
                        deskewFlag = 1;
                        break;
                    }
                }

                if (deskewFlag != 1) {
                    #pragma omp parallel for num_threads(numberOfCores)
                    for (int h = 0; h < laserCloudIn->height - 1; ++h) {
                        for (int w = 0; w < laserCloudIn->width - 1; ++w) {
                            laserCloudIn->at(w, h).timestamp = 3.236*((((h+1)%64==0?64:(h+1)%64)-1)/4.f) * 1e-6 + w * 55.552 * 1e-6;
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

            static int ringFlag = 0;
            if (ringFlag == 0)
            {
                ringFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == "ring")
                    {
                        ringFlag = 1;
                        break;
                    }
                }
                // if (ringFlag == -1)
                // {
                //     ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                //     ros::shutdown();
                // }
            }   

            if (deskewFlag == 0)
            {
                deskewFlag = -1;
                for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
                {
                    if (currentCloudMsg.fields[i].name == timeField)
                    {
                        deskewFlag = 1;
                        break;
                    }
                }
                // if (deskewFlag == -1)
                //     ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
            }

            //origin
            if(deskewFlag == -1 || ringFlag == -1)
            {

                
                #pragma omp parallel for num_threads(numberOfCores)
                for(int h=0;h<laserCloudIn->height-1;++h){
                    for(int w=0;w<laserCloudIn->width-1;++w){
                        laserCloudIn->at(w,h).ring=h;
                    }
                } 

                if (deskewFlag == 0) {
                    deskewFlag = -1;
                    for (int i = 0; i < (int) currentCloudMsg.fields.size(); ++i) {
                        if (currentCloudMsg.fields[i].name == timeField) {
                            deskewFlag = 1;
                            break;
                        }
                    }

                    if (deskewFlag != 1) {
                        #pragma omp parallel for num_threads(numberOfCores)
                        for (int h = 0; h < laserCloudIn->height - 1; ++h) {
                            for (int w = 0; w < laserCloudIn->width - 1; ++w) {
                                laserCloudIn->at(w, h).timestamp = (w*55.52+h%16*2.88+1.44*float(h/16))*1e-6;
                            }
                        }
                        deskewFlag = 1;
                    }
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

        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
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

            if (currentImuTime > timeScanNext + 0.01)
                break;

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

            if (currentOdomTime > timeScanNext + 0.01)
                break;

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

        double pointTime =  relTime;

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

    int rowIdn_max=0,rowIdn_min=999;
    void projectPointCloud()
    {
        int rowIdn;

        float verticalAngle, horizonAngle;
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        static float ang_res_y = Vertical_angle/float(N_SCAN-1);

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

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].timestamp - laserCloudIn->points[0].timestamp); 

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

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jueying_slam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}