#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/obj_debug", 10);

    double speed = 0.5; // m/s
    double max_dist = 1.5;
    double min_dist = 0.0;
    int direction = 1; // 1: forward, -1: backward
    ros::Rate rate(10); // 10Hz
    double dt = 0.1; // 1/10Hz
    double dist = 0.0; // 沿斜线的距离

    while (ros::ok())
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = dist;
        pose.pose.position.y = dist; // 斜线 y = x
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0; // 单位四元数

        pub.publish(pose);

        dist += direction * speed * dt;
        if (dist >= max_dist) {
            dist = max_dist;
            direction = -1;
        } else if (dist <= min_dist) {
            dist = min_dist;
            direction = 1;
        }

        rate.sleep();
    }
    return 0;
}