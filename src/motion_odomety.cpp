#include <ros/ros.h>
#include "ekf_test/Input.h"
#include "geometry_msgs/Pose2D.h"


class ModelOdometry {
 private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    bool isStart;
    float a1, a2, a3, a4;
    float x, y , theta;
    float x_pre, y_pre, theta_pre;
    float delta_r1, delta_r2, delta_trans;
    float delta_hat_r1, delta_hat_r2, delta_hat_trans;
    std::default_random_engine generator;

 public:
    explicit ModelOdometry(bool start) : isStart(start) {}
    ~ModelOdometry() {}

    void dataCallback(const ekf_test::Input::ConstPtr& msg) {
        geometry_msgs::Pose2D pose = msg->odom;
        if (!isStart) {
            // Algorithm motion_model_odometry

            // deltas
            delta_r1 = std::atan2(
                (y_pre - pose.y), x_pre - pose.x) - pose.theta;
            delta_trans = std::sqrt(
                pow(2, (pose.x - x_pre)) + pow(2, (pose.y - y_pre)));
            delta_r2 = pose.theta - theta_pre - delta_r1;
        } else {
            x_pre = pose.x;
            y_pre = pose.y;
            theta_pre = pose.theta;
            isStart = false;
        }
    }
};



