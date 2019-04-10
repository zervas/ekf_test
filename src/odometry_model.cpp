// Copyright
//
// TODO(mitsos)

#include <ros/ros.h>
#include "ekf_test/Input.h"
#include "geometry_msgs/Pose2D.h"



class MotionModel {
 private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    bool isStart;
    float a1, a2, a3, a4;
    float x, y, theta;
    float x_pre, y_pre, theta_pre;
    float delta_r1, delta_r2, delta_trans;
    float delta_hat_r1, delta_hat_r2, delta_hat_trans;

    std::default_random_engine generator;

 public:
    explicit MotionModel(std::string, std::string);
    ~MotionModel();
    void dataSampleCallback(const ekf_test::Input::ConstPtr &);
};

MotionModel::MotionModel(std::string sub, std::string pub) {
        isStart = true;
        a1 = 0.1;
        a2 = 0.1;
        a3 = 0.1;
        a4 = 0.1;
        sub_ = nh_.subscribe(sub, 10, &MotionModel::dataSampleCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Pose2D>(pub, 10);
}

MotionModel::~MotionModel() {
    // TODO(mitsos)
}

void MotionModel::dataSampleCallback(const ekf_test::Input::ConstPtr& msg) {
    // Gernerate motion model samples from sensor data
    geometry_msgs::Pose2D pose = msg->odom;
    if (!isStart) {
        // deltas
        delta_r1 = std::atan2((y_pre - pose.y), x_pre - pose.x) - pose.theta;
        delta_trans = std::sqrt(
            pow(2, (pose.x - x_pre)) + pow(2, (pose.y - y_pre)));
        delta_r2 = pose.theta - theta_pre - delta_r1;

        // normal distributions
        float b1 = a1 * std::abs(delta_r1) + a2 * std::abs(delta_trans);
        float b2 = a3 * std::abs(delta_trans) +
            a4 * std::abs(delta_r1 + delta_r2);
        float b3 = a1 * std::abs(delta_r2) + a2 * std::abs(delta_trans);

        std::normal_distribution<float> epsilon_1(0, b1);
        std::normal_distribution<float> epsilon_2(0, b2);
        std::normal_distribution<float> epsilon_3(0, b3);

        // delta hats
        delta_hat_r1 = delta_r1 - epsilon_1(generator);
        delta_hat_trans = delta_trans - epsilon_2(generator);
        delta_hat_r2 = delta_r2 - epsilon_3(generator);

        // new pose estimate
        geometry_msgs::Pose2D new_pose;
        new_pose.x = x_pre + (delta_hat_trans * cos(theta_pre + delta_hat_r1));
        new_pose.y = y_pre + (delta_hat_trans * sin(theta_pre + delta_hat_r1));
        new_pose.theta = theta_pre + delta_hat_r1 + delta_hat_r2;
        // Publish it
        pub_.publish(new_pose);


    } else {
        x_pre = pose.x;
        y_pre = pose.y;
        theta_pre = pose.theta;
        isStart = false;
    }
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_model");

    MotionModel odom_model("csv_data", "odom_data");

    ros::spin();
    return 0;
}