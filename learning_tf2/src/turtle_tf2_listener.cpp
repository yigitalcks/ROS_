#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle n;

    ros::Publisher turtle_vel = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    ros::Publisher transform_val = n.advertise<geometry_msgs::TransformStamped>("transform", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);

    while (n.ok()){
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        geometry_msgs::Twist vel_msg;

        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

        turtle_vel.publish(vel_msg);
        transform_val.publish(transformStamped);
        rate.sleep();
    }


    return 0;
}