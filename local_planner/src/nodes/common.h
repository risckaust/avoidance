#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace avoidance {

float distance2DPolar(int e1, int z1, int e2, int z2);
Eigen::Vector3f fromPolarToCartesian(int e, int z, double radius,
                                     const geometry_msgs::Point& pos);
double indexAngleDifference(int a, int b);
geometry_msgs::Vector3Stamped getWaypointFromAngle(
    int e, int z, const geometry_msgs::Point& pos);
bool hasSameYawAndAltitude(const geometry_msgs::PoseStamped& old_wp,
                           const geometry_msgs::Vector3Stamped& new_wp,
                           double new_yaw, double old_yaw);
double elevationIndexToAngle(int e, double res);
double azimuthIndexToAngle(int z, double res);
int azimuthAnglefromCartesian(const Eigen::Vector3f& position,
                              const Eigen::Vector3f& origin);
int azimuthAnglefromCartesian(double x, double y, const Eigen::Vector3f& pos);
int elevationAnglefromCartesian(const Eigen::Vector3f& pos,
                                const Eigen::Vector3f& origin);
int elevationAnglefromCartesian(double x, double y, double z,
                                const Eigen::Vector3f& pos);
int elevationAngletoIndex(int e, int res);
int azimuthAngletoIndex(int z, int res);

double nextYaw(const geometry_msgs::PoseStamped& u,
               const geometry_msgs::Point& v);

geometry_msgs::PoseStamped createPoseMsg(const geometry_msgs::Point& waypt,
                                         double yaw);

double velocitySigmoid(double max_vel, double min_vel, double slope,
                       double v_old, double elapsed);
double velocityLinear(double max_vel, double min_vel, double slope,
                      double v_old, double elapsed);
void wrapAngleToPlusMinusPI(double& angle);
double getAngularVelocity(double desired_yaw, double curr_yaw);

Eigen::Vector3f toEigen(const geometry_msgs::Point& p);
Eigen::Vector3f toEigen(const pcl::PointXYZ& xyz);
Eigen::Vector3f toEigen(const geometry_msgs::Point& p);

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3);
pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3);
geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3);
}

#endif  // COMMON_H
