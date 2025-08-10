#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandLong.h> 
//#include "waypoints.h"
#include <mavros_msgs/PositionTarget.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <mission_node/Bounding_box.h>
#include <mission_node/class_pub.h>
#include <deque>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <map>

#ifndef _INPUT_H
#define _INPUT_H
class StateSub
{
public:
    mavros_msgs::State state_code ;
    void feed(const mavros_msgs::State::ConstPtr& msg);
};
class PoseSub
{
public:
    geometry_msgs::PoseStamped pose_local;
    void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

class QrSub
{
public:
    std::vector<std::string> datas;
    std_msgs::String qr_data;
    void Qr_cb(const std_msgs::String::ConstPtr& msg);
};
class ImageSub
{
public:

    mission_node::Bounding_box image_data;
    void Image_cb(const mission_node::Bounding_box::ConstPtr& msg);
};
class ClassifySub
{
public:
    // 方法1: 滑动窗口
    std::deque<std::string> classification_window;

    std::map<std::string, std::string> string_mapping = {
        {"camel", "elephant"},
        {"beaver", "cattle"},
        {"bear","cattle"},
        {"bicycle", "crab"},
        {"spider","crab"},
        {"rose", "poppy"},
        {"rocket","pickup_truck"},
        {"skyscraper", "streetcar"},
        {"dolphin","whale"}
    };

    mission_node::class_pub class_data;
    void Classify_cb(const mission_node::class_pub::ConstPtr& msg);

    std_msgs::String classfiy_data;
    int confidence_ =0 ;

};
class CircleSub
{
public:
    geometry_msgs::PoseStamped circle_pose;
    std::vector<geometry_msgs::Point> ring_center_buffer;
    void circle_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool judge_circle_center = false;
};
class ObjSub
{
public:
    geometry_msgs::PoseStamped obj_position;
    void Obj_posSub(const geometry_msgs::PoseStamped::ConstPtr& msg);
};
class TrajSub
{
public:
    std_msgs::UInt8 traj_sub;
    void Traj_Sub(const std_msgs::UInt8::ConstPtr& msg);
};
class DynamicSub
{
public:
    std_msgs::Bool dynamic_judge;
    void Dynamic_CB(const std_msgs::Bool::ConstPtr& msg);

};
#endif