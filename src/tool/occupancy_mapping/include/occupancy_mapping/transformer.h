#ifndef OCCUPANCY_MAPPING_TRANSFORMER_H_
#define OCCUPANCY_MAPPING_TRANSFORMER_H_

#include <string>
#include <queue>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

namespace occupancy_mapping
{

    class Transformer
    {
    public:
        Transformer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        int lookupTransform(const std::string &from_frame, const std::string &to_frame, const ros::Time &timestamp, geometry_msgs::PoseStamped *transform);

        void odometryCallback(const nav_msgs::Odometry &transform_msg);

    private:
        int lookupTransformTf(const std::string &from_frame, const std::string &to_frame, const ros::Time &timestamp, geometry_msgs::PoseStamped *transform);

        int lookupTransformQueue(const ros::Time &timestamp, geometry_msgs::PoseStamped *transform);

        void transformTFToPoseStamped(tf::StampedTransform tf_transform, geometry_msgs::PoseStamped *pose_stamped);

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        std::string map_frame_;
        std::string sensor_frame_;

        std::string odom_topic_;

        bool use_tf_transforms_;

        double time_threshold_;

        tf::TransformListener* tf_listener_;

        // l Only used if use_tf_transforms_ set to false.
        ros::Subscriber odom_sub_;

        std::queue<geometry_msgs::PoseStamped> pose_queue_;
    };

} // namespace

#endif
