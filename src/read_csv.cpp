// Copyright
//
// TODO

#include <cstdlib>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "ekf_test/read_csv.h"

std::istream & operator>> (std::istream &in, CSVRow &data) {
    data.read_next_row(in);
    return in;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "CSV_reader");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>(
        "pose2D_publisher", 10);


    // * Read the CSV file here
    std::ifstream fin;
    fin.open("/home/nextgen/catkin_ws/src/ekf_test/src/test.csv");

    ros::Rate loop_rate(1);
    CSVRow row;

    while (pub.getNumSubscribers() < 1) {
        continue;
    }

    while (fin >> row) {
        static geometry_msgs::Pose2D pose;      // Should I 'static'?
        pose.x = atof(row[3].c_str());
        pose.y = atof(row[5].c_str());
        pose.theta = atof(row[7].c_str());
        pub.publish(pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Exiting!");
    return 0;
}