#include <urdf/model.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

int main(int argc, char** argv) {

    ros::init(argc, argv, "parser");
    if(argc != 2) {
        ROS_ERROR("Need a urdf file");
        return -1;
    }

    std::string urdf_name = argv[1];
    KDL::Tree my_tree;

    urdf::Model model;

        if (!model.initFile(urdf_name)){
            ROS_ERROR("Failed to parse urdf file");
            return -1;
        }
        if (!kdl_parser::treeFromUrdfModel(model, my_tree)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }

    ROS_INFO("Successfully parsed urdf file");







    return 0;
}