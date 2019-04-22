// Copyright
//
// TODO

#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ekf_test/read_csv.h"
#include "ekf_test/Input.h"
#include "ekf_test/EKFMotion.h"
#include "ekf_test/RangeBearing.h"
#include <utility>

std::istream & operator>> (std::istream &in, CSVRow &data) {
    data.read_next_row(in);
    return in;
}

void clean_msg(ekf_test::Input& msg) {
    ekf_test::Input new_msg;
    msg = std::move(new_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CSV_reader");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<ekf_test::Input>(
        "csv_data", 100);


    // * Read the CSV file here
    std::ifstream fin;
    // fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/test.csv");
    fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/sensor_data.dat");
    // fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/sensor_data.csv");

    ros::Rate loop_rate(10);
    CSVRow row;

    while (pub.getNumSubscribers() < 1) {
        continue;
    }

    float x0 = 0.0;
    float y0 = 0.0;
    float theta0 = 0.0;

    static ekf_test::Input msg;
    std::vector<ekf_test::RangeBearing> sensor_data;
    while (fin >> row) {
        if ((row[0] == "ODOMETRY")) {
            // publish the previous message
            msg.sensors = sensor_data;
            pub.publish(msg);
            clean_msg(msg);
            sensor_data.clear();
            // get the odom input
            msg.motion.delta_r1 = atof(row[1].c_str());
            msg.motion.delta_trans = atof(row[2].c_str());
            msg.motion.delta_r2 = atof(row[3].c_str());
        } else if (row[0] == "SENSOR") {
            ROS_INFO("%s ", row[0].c_str());
            // Get the sensor index
            ekf_test::RangeBearing landmark;
            landmark.id = atoi(row[1].c_str());
            landmark.range = atof(row[2].c_str());
            landmark.phi = atof(row[3].c_str());
            sensor_data.push_back(landmark);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Exiting!");
    return 0;
}