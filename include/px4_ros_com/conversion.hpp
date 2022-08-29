#include <Eigen/core>




inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

inline void msgMultiDofJointTrajectoryFromPositionYaw(
    const Eigen::Vector3d& position, double yaw,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  EigenTrajectoryPoint point;
  point.position_W = position;
  point.setFromYaw(yaw);

  msgMultiDofJointTrajectoryFromEigen(point, msg);
}

