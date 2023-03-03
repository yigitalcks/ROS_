#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "point_publisher");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

    unsigned int num_points {100};

    int count = 0;
    ros::Rate rate(1.0);

    while (ros::ok()) {
        sensor_msgs::PointCloud point;
        point.header.frame_id = "base_laser";
        point.header.stamp = ros::Time::now();

        point.points.resize(num_points);
        point.channels.resize(1);
        point.channels.at(0).name = "intensities";
        point.channels.at(0).values.resize(num_points);

        for(unsigned int i = 0; i < num_points; ++i){
            point.points[i].x = 1 + count;
            point.points[i].y = 2 + count;
            point.points[i].z = 3 + count;
            point.channels[0].values[i] = 100 + count;
        }
        pub.publish(point);
        ++count;
        rate.sleep();
    }
}