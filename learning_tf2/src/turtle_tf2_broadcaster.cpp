#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped ;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = turtle_name;
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0.0;
     
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "my_tf2_broadcaster");
    
    ros::NodeHandle n_p("~");
    if (n_p.hasParam("turtle_name")) {
        n_p.getParam("turtle_name", turtle_name);
    }
    else {
        ROS_WARN("turtle name has not found");
    }
    ros::NodeHandle n;

    if(turtle_name == "turtle2") {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = n.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = atof(argv[1]);
        turtle.request.y = atof(argv[2]);
        turtle.request.theta = 0;
        turtle.request.name = turtle_name;
        spawner.call(turtle);
    }

    ros::Subscriber sub = n.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();

    return 0;
}
