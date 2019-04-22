// Copyright
//
// TODO(mitsos)

#include <ros/ros.h>
#include "ekf_test/Input.h"
#include "ekf_test/RangeBearing.h"
#include "geometry_msgs/Pose2D.h"
#include <Eigen/Dense>
#include <vector>


class MotionModel {
    typedef Eigen::Matrix<float, Eigen::Dynamic, 1> state;

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

    Eigen::MatrixXf fx;
    int num_landmarks;
    std::default_random_engine generator;
    state state_vector;

    Eigen::MatrixXf sigma, sigma_hat;

    // std::vector<RangeBearing> get_measuremets(geometry_msgs::Point sensors[8]);

 public:
    explicit MotionModel(std::string, std::string);
    ~MotionModel();
    void dataSampleCallback(const ekf_test::Input::ConstPtr &);
    void init_matrices(int N);
    void updateStateSpace(const ekf_test::Input::ConstPtr& msg);
    void updateStateSpace(float delta_r1, float delta_trans, float delta_r2);
    void updateCovariance(const ekf_test::Input::ConstPtr& msg);
    void predictionStep(const ekf_test::Input::ConstPtr& msg);
};

MotionModel::MotionModel(std::string sub, std::string pub) {
        isStart = true;
        a1 = 0.5;
        a2 = 0.1;
        a3 = 0.05;
        a4 = 0.1;
        sub_ = nh_.subscribe(sub, 10, &MotionModel::dataSampleCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Pose2D>(pub, 100);

        init_matrices(3);
}

MotionModel::~MotionModel() {
    // TODO(mitsos)
}

void MotionModel::init_matrices(int N) {
    // * N: predifined number of landmarks
    // Initialize the state vector (first step is set to zeros)
    state_vector = state(3+2*N);
    state_vector.setZero();
    // Matrix to map the odom equations to 2N+3 dim space
    fx = Eigen::MatrixXf::Zero(3, 3+2*N);
    fx.block<3, 3>(0, 0) = Eigen::MatrixXf::Identity(3, 3);

    sigma = Eigen::MatrixXf(3+2*N, 3+2*N);
    sigma_hat = Eigen::MatrixXf(3+2*N, 3+2*N);
    sigma.topLeftCorner(3, 3) = Eigen::MatrixXf::Identity(3, 3);
    sigma.bottomRightCorner(2*N, 2*N) =
        1000 * Eigen::MatrixXf::Identity(2*N, 2*N);
    // Full state Jacobian matrix (Sigma_hat = J * Sigma * J_trans + Rt)


}

void MotionModel::dataSampleCallback(const ekf_test::Input::ConstPtr& msg) {
    // updateStateSpace(msg);
    // updateCovariance(msg);
    predictionStep(msg);
}

void MotionModel::predictionStep(const ekf_test::Input::ConstPtr& msg) {
    float delta_r1 = msg->motion.delta_r1;
    float delta_trans = msg->motion.delta_trans;
    float delta_r2 = msg->motion.delta_r2;
    const std::vector<ekf_test::RangeBearing> sensor_data = msg->sensors;

    updateStateSpace(delta_r1, delta_trans, delta_r2);
}

void MotionModel::updateStateSpace(float delta_r1, float delta_trans,
                                   float delat_r2) {
    if (!isStart) {
        // normal distributions
        float b1 = a1 * std::abs(delta_r1) + a2 * std::abs(delta_trans);
        float b2 = a3 * std::abs(delta_trans) +
            a4 * std::abs(delta_r1 + delta_r2);
        float b3 = a1 * std::abs(delta_r2) + a2 * std::abs(delta_trans);

        // delta hats
        std::normal_distribution<float> epsilon_1(0, b1);
        std::normal_distribution<float> epsilon_2(0, b2);
        std::normal_distribution<float> epsilon_3(0, b3);

        delta_hat_r1 = delta_r1 - epsilon_1(generator);
        delta_hat_trans = delta_trans - epsilon_2(generator);
        delta_hat_r2 = delta_r2 - epsilon_3(generator);

        Eigen::Vector3f odom(3);
        odom << x_pre + (delta_hat_trans * cos(theta_pre + delta_hat_r1)),
                y_pre + (delta_hat_trans * sin(theta_pre + delta_hat_r1)),
                theta_pre + delta_hat_r1 + delta_hat_r2;

        // Prediction step (Motion)
        state_vector = state_vector + fx.transpose() * odom;
        geometry_msgs::Pose2D new_pose;
        new_pose.x = state_vector(0);
        new_pose.y = state_vector(1);
        new_pose.theta = state_vector(2);
        // Publish it
        pub_.publish(new_pose);

        x_pre = state_vector(0);
        y_pre = state_vector(1);
        theta_pre = state_vector(2);

    } else {
        // x0, y0, theta0 are zeros. It's safe to use the first odometry
        // that is also all zeros (Note! Odom is in deltas not in x,y,theta)
        x_pre = delta_r1;
        y_pre = delta_trans;
        theta_pre = delta_r2;
        isStart = false;
    }
}

void MotionModel::updateStateSpace(const ekf_test::Input::ConstPtr& msg) {
    // float delta_r1 = msg->motion.delta_r1;
    // float delta_trans = msg->motion.delta_trans;
    // float delta_r2 = msg->motion.delta_r2;
    if (!isStart) {
        // normal distributions
        float b1 = a1 * std::abs(delta_r1) + a2 * std::abs(delta_trans);
        float b2 = a3 * std::abs(delta_trans) +
            a4 * std::abs(delta_r1 + delta_r2);
        float b3 = a1 * std::abs(delta_r2) + a2 * std::abs(delta_trans);

        // delta hats
        std::normal_distribution<float> epsilon_1(0, b1);
        std::normal_distribution<float> epsilon_2(0, b2);
        std::normal_distribution<float> epsilon_3(0, b3);

        delta_hat_r1 = delta_r1 - epsilon_1(generator);
        delta_hat_trans = delta_trans - epsilon_2(generator);
        delta_hat_r2 = delta_r2 - epsilon_3(generator);

        Eigen::Vector3f odom(3);
        odom << x_pre + (delta_hat_trans * cos(theta_pre + delta_hat_r1)),
                y_pre + (delta_hat_trans * sin(theta_pre + delta_hat_r1)),
                theta_pre + delta_hat_r1 + delta_hat_r2;

        // Prediction step (Motion)
        state_vector = state_vector + fx.transpose() * odom;
        geometry_msgs::Pose2D new_pose;
        new_pose.x = state_vector(0);
        new_pose.y = state_vector(1);
        new_pose.theta = state_vector(2);
        // Publish it
        pub_.publish(new_pose);

        x_pre = state_vector(0);
        y_pre = state_vector(1);
        theta_pre = state_vector(2);

    } else {
        // x0, y0, theta0 are zeros. It's safe to use the first odometry
        // that is also all zeros (Note! Odom is in deltas not in x,y,theta)
        x_pre = msg->motion.delta_r1;
        y_pre = msg->motion.delta_trans;
        theta_pre = msg->motion.delta_r2;
        isStart = false;
    }
}

void MotionModel::updateCovariance(const ekf_test::Input::ConstPtr& msg) {
    /*
        Sigma_xx(3, 3),     Sigma_xm(3, 2*N)
        Sigma_mx(2*N, 3),   Sigma_mm(2*N, 2*N)
    */
    if (isStart == false) {
        float delta_r1 = msg->motion.delta_r1;
        float delta_trans = msg->motion.delta_trans;
        float delta_r2 = msg->motion.delta_r2;

        // float g1 = - delta_trans * sin(theta_pre + delta_r1);
        // float g2 = delta_trans * cos(theta_pre + delta_r1);
        Eigen::MatrixXf jacob(3, 3);
        jacob.setIdentity();
        jacob(0, 2) = - delta_trans * sin(theta_pre + delta_r1);
        jacob(1, 2) = delta_trans * cos(theta_pre + delta_r1);

        int twoN = sigma.rows() - 3;
        // Sigma_xx
        sigma_hat.block<3, 3>(0, 0) =
            jacob * sigma.block<3, 3>(0, 0) * jacob.transpose();
        // Sigma_xm
        sigma_hat.block(0, 3, 3, twoN) = jacob * sigma.block(0, 3, 3, twoN);
        // Sigma_mx
        sigma_hat.block(3, 0, twoN, 3) =
            sigma_hat.block(0, 3, 3, twoN).transpose();
        // Sigma_mm
        // Untouched!

        // Rt
        float motion_noise = 0.1;
        Eigen::MatrixXf Rt(3+twoN, 3+twoN);
        Rt.topLeftCorner(3, 3) = Eigen::MatrixXf::Identity(3, 3) * motion_noise;
        Rt(2, 2) = 0.1 / 10;

        sigma_hat += Rt;
    }
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_model");

    MotionModel odom_model("csv_data", "odom_data");

    ros::spin();
    return 0;
}