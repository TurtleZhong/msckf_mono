//
// Copyright (C) 2017 Leo Koppel <lkoppel@uwaterloo.ca>
//


#include <nodelet/loader.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_tracking_node");
    // Create a new instance of your nlp
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    // namespace name / library name
    nodelet.load(nodelet_name,
                 "feature_tracker/feature_tracking_nodelet", remap,
                 nargv);
    ros::spin();
    return 0;
}
