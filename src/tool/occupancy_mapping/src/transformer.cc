#include "occupancy_mapping/transformer.h"

namespace occupancy_mapping
{

    Transformer::Transformer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          map_frame_("map"),
          sensor_frame_(""),
          odom_topic_("Odometry"),
          time_threshold_(10000000),
          use_tf_transforms_(true)
    {
        nh_private_.param("/occupancy_mapping_2D/map_frame", map_frame_, map_frame_);
        nh_private_.param("/occupancy_mapping_2D/sensor_frame", sensor_frame_, sensor_frame_);
        nh_private_.param("/occupancy_mapping_2D/odom_topic", odom_topic_, odom_topic_);
        nh_private_.param("/occupancy_mapping_2D/use_tf_transforms", use_tf_transforms_, use_tf_transforms_);
        nh_private_.param("/occupancy_mapping_2D/time_threshold", time_threshold_, time_threshold_);
        tf_listener_ = new tf::TransformListener(ros::Duration(30));
        if (!use_tf_transforms_)
        {
            odom_sub_ = nh_.subscribe(odom_topic_, 40, &Transformer::odometryCallback, this);
        }
    }
    ////
    void Transformer::odometryCallback(const nav_msgs::Odometry &odom_msg)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = odom_msg.pose.pose.position.x;
        pose_stamped.pose.position.y = odom_msg.pose.pose.position.y;
        pose_stamped.pose.position.z = odom_msg.pose.pose.position.z;
        pose_stamped.pose.orientation = odom_msg.pose.pose.orientation;

        pose_stamped.header.stamp = odom_msg.header.stamp;
        pose_queue_.push(pose_stamped);
    }

    int Transformer::lookupTransform(const std::string &from_frame,
                                     const std::string &to_frame,
                                     const ros::Time &timestamp,
                                     geometry_msgs::PoseStamped *transform)
    {
        if (use_tf_transforms_)
        {
            return lookupTransformTf(from_frame, to_frame, timestamp, transform);
        }
        else
        {
            return lookupTransformQueue(timestamp, transform);
        }
    }

    void Transformer::transformTFToPoseStamped(tf::StampedTransform tf_transform, geometry_msgs::PoseStamped *pose_stamped)
    {
        geometry_msgs::PoseStamped stPose;
        geometry_msgs::Quaternion quat;
        tf::Quaternion tfQuat;
        tfQuat = tf_transform.getRotation();
        quat.x = tfQuat.x();
        quat.y = tfQuat.y();
        quat.z = tfQuat.z();
        quat.w = tfQuat.w();
        pose_stamped->pose.orientation = quat;

        tf::Vector3 tfVec;
        tfVec = tf_transform.getOrigin();
        pose_stamped->pose.position.x = tfVec.getX();
        pose_stamped->pose.position.y = tfVec.getY();
        pose_stamped->pose.position.z = tfVec.getZ();
        pose_stamped->header.frame_id = tf_transform.frame_id_; // 坐标系
        pose_stamped->header.stamp = tf_transform.stamp_;       // 保留原始转换的时间戳
    }

    int Transformer::lookupTransformTf(const std::string &from_frame,
                                       const std::string &to_frame,
                                       const ros::Time &timestamp,
                                       geometry_msgs::PoseStamped *transform)
    {
        tf::StampedTransform tf_transform;
        ros::Time time_to_lookup = timestamp;

        // Allow overwriting the TF frame for the sensor.
        std::string to_frame_modified = to_frame;
        if (!sensor_frame_.empty())
        {
            to_frame_modified = sensor_frame_;
        }
        // if (timestamp < (ros::Time::now() - tf_listener_->getCacheLength())) // tf缓冲区中最老的tf时间戳，比点云队列中最老的时间戳还老
        // {
        //     std::cout << "Lidar timestamp is older than cache in tf_listener! Drop this lidar pointcloud!" << std::endl;
        //     std::cout<<std::fixed<<std::setprecision(9)<< timestamp.toSec() <<", "<<ros::Time::now().toSec()<<", "<<(ros::Time::now() - tf_listener_->getCacheLength()).toSec()<<std::endl;
        //     return 2; // 需要剔除最老的点云，否则这帧点云会永远查不到其对应的tf
        // }
        if (!tf_listener_->canTransform(from_frame, to_frame_modified, time_to_lookup))
        {
            // std::cout << "can not transform" << std::endl;

            return 0;
        }

        try
        {
            // tf_listener_.canTransform(from_frame, to_frame_modified, time_to_lookup);
            tf_listener_->lookupTransform(from_frame, to_frame_modified, time_to_lookup, tf_transform);
        }
        catch (tf::TransformException &ex)
        { // NOLINT
            ROS_ERROR_STREAM("[Occypancy mapping] Error getting TF transform "
                             << "between " << from_frame << " and " << to_frame_modified << " : " << ex.what());

            return 0;
        }

        transformTFToPoseStamped(tf_transform, transform);
        return true;
    }

    int Transformer::lookupTransformQueue(const ros::Time &timestamp,
                                          geometry_msgs::PoseStamped *transform)
    {
        if (pose_queue_.empty())
        {
            ROS_WARN_STREAM_THROTTLE(30, "[Occypancy mapping] No match found for transform timestamp: "
                                             << timestamp
                                             << " as transform queue is empty.");
            return false;
        }
        geometry_msgs::PoseStamped transform_last = pose_queue_.front();
        if (transform_last.header.stamp.toSec() - time_threshold_ > timestamp.toSec())
        {
            std::cout << "Lidar timestamp is older than cache in tf_listener! Drop this lidar pointcloud!" << std::endl;
            return 2;
        }
        while (!pose_queue_.empty())
        {

            *transform = pose_queue_.front();
            if (pose_queue_.front().header.stamp > timestamp)
            {
                // pose_queue_.pop();
                break;
            }
            transform_last = pose_queue_.front();
            pose_queue_.pop();
        }
        if (abs((transform->header.stamp - timestamp).toSec()) < abs((transform_last.header.stamp - timestamp).toSec()))
        { // 时间足够接近
            if (abs((transform->header.stamp - timestamp).toSec()) < time_threshold_)
            {
                return 1;
            }
            else
            {
                std::cout<<std::fixed<<std::setprecision(9) << "[Occypancy mapping] Time between lidar "<<timestamp.toSec()<<" and newer odom "<<transform->header.stamp.toSec()<<" too long" << std::endl;
                return 0;
            }
        }
        else
        {
            if (abs((transform_last.header.stamp - timestamp).toSec()) < time_threshold_)
            {
                *transform = transform_last;
                return 1;
            }
            else
            {
                std::cout<<std::fixed<<std::setprecision(9) << "[Occypancy mapping] Time between lidar "<<timestamp.toSec()<<" and older odom "<<transform_last.header.stamp.toSec()<<" too long" << std::endl;
                return 0;
            }
        }

        // return true;
    }

} //
