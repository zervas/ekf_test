// Copyright
//
// TODO

#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ekf_test/read_csv.h"
#include "ekf_test/Input.h"
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
        "csv_data", 10);


    // * Read the CSV file here
    std::ifstream fin;
    fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/test.csv");
    // fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/sensor_data.csv");

    ros::Rate loop_rate(2);
    CSVRow row;

    while (pub.getNumSubscribers() < 1) {
        continue;
    }

    float x0 = 0.0;
    float y0 = 0.0;
    float theta0 = 0.0;

    static ekf_test::Input msg;
    while (fin >> row) {
        ROS_INFO("%s ", row[0].c_str());
        if ((row[0] == "ODOMETRY")) {
            // publish the previous message
            pub.publish(msg);
            clean_msg(msg);
            // get the odom input
            msg.odom.x = atof(row[1].c_str());
            msg.odom.y = atof(row[2].c_str());
            msg.odom.theta = atof(row[3].c_str());
        } else if (row[0] == "SENSOR") {
            // Get the sensor index
            int index = atoi(row[1].c_str());
            msg.sensors[index-1].x = atof(row[2].c_str());
            msg.sensors[index-1].y = atof(row[3].c_str());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Exiting!");
    return 0;
}