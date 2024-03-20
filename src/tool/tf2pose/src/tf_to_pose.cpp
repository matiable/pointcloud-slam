#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

struct pose {
  double x, y, z;
  double pitch, roll, yaw;
  pose() {}
  pose(double x, double y, double z, double roll, double pitch, double yaw)
      : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
  void reset() {
    x = 0.;
    y = 0.;
    z = 0.;
    roll = 0.;
    pitch = 0.;
    yaw = 0.;
  }
  pose operator+(const pose &b) const {
    return pose(x + b.x, y + b.y, z + b.z, roll + b.roll, pitch + b.pitch,
                yaw + b.yaw);
  }
  pose operator-(const pose &b) const {
    return pose(x - b.x, y - b.y, z - b.z, roll - b.roll, pitch - b.pitch,
                yaw - b.yaw);
  }
  pose operator+=(const pose &b) { return *this + b; }
  pose operator-=(const pose &b) { return *this - b; }
};

bool pose2GeometryPose(geometry_msgs::Pose &to, const pose &from) {
  to.position.x = from.x;
  to.position.y = from.y;
  to.position.z = from.z;
  to.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(from.roll, from.pitch, from.yaw);

  return true;
}

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "tf_to_pose");

  // 创建节点句柄
  ros::NodeHandle node;

  // 创建发布turtle2速度控制指令的发布者
  ros::Publisher pub_current_pose_with_cov =
      node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1);

  // 创建tf的监听器
  tf::TransformListener listener;

  ros::Rate rate(500.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      listener.waitForTransform("/map", "/base_link", ros::Time(0),
                                ros::Duration(3.0));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Isometry3d map_to_base_eigen;
    tf::transformTFToEigen(transform, map_to_base_eigen);

    pose robot_pose;
    tf::Matrix3x3 R;
    R.setValue(static_cast<double>(map_to_base_eigen(0, 0)),
               static_cast<double>(map_to_base_eigen(0, 1)),
               static_cast<double>(map_to_base_eigen(0, 2)),
               static_cast<double>(map_to_base_eigen(1, 0)),
               static_cast<double>(map_to_base_eigen(1, 1)),
               static_cast<double>(map_to_base_eigen(1, 2)),
               static_cast<double>(map_to_base_eigen(2, 0)),
               static_cast<double>(map_to_base_eigen(2, 1)),
               static_cast<double>(map_to_base_eigen(2, 2)));
    robot_pose.x = map_to_base_eigen(0, 3);
    robot_pose.y = map_to_base_eigen(1, 3);
    robot_pose.z = map_to_base_eigen(2, 3);
    R.getRPY(robot_pose.roll, robot_pose.pitch, robot_pose.yaw);

    geometry_msgs::PoseWithCovarianceStamped msg_current_pose_with_cov;
    pose2GeometryPose(msg_current_pose_with_cov.pose.pose, robot_pose);

    msg_current_pose_with_cov.header.stamp = ros::Time::now();
    msg_current_pose_with_cov.header.frame_id = "map";
    msg_current_pose_with_cov.pose.covariance[0] = pow(0.01, 2);
    msg_current_pose_with_cov.pose.covariance[7] = pow(0.01, 2);
    msg_current_pose_with_cov.pose.covariance[14] = pow(0.01, 2);
    msg_current_pose_with_cov.pose.covariance[21] = pow(0.01, 2);
    msg_current_pose_with_cov.pose.covariance[28] = pow(0.01, 2);
    msg_current_pose_with_cov.pose.covariance[35] = pow(0.01, 2);
    pub_current_pose_with_cov.publish(msg_current_pose_with_cov);

    rate.sleep();
  }
  return 0;
};
