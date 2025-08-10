#include "ros/ros.h"
#include "MissionFsm.h"
#include <boost/bind.hpp>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "run_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    MissionFSM fsm;  // 声明 MissionFSM 类的实例
    // 订阅状态主题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,boost::bind(&StateSub::feed, &fsm.state_mission, _1));
    ros::Subscriber iamge_sub = nh.subscribe<mission_node::Bounding_box>("yolo/bounding_box",10,boost::bind(&ImageSub::Image_cb, &fsm.image_staff, _1));
    ros::Subscriber class_sub = nh.subscribe<mission_node::class_pub>("/yolo/classify",10,boost::bind(&ClassifySub::Classify_cb, &fsm.class_staff, _1));

    // 订阅位置主题
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10, boost::bind(&PoseSub::local_pose_cb, &fsm.pose_data, _1));
    ros::Subscriber circle_sub = nh.subscribe<geometry_msgs::PoseStamped>("/circle_position",10, boost::bind(&CircleSub::circle_pose_cb, &fsm.circle_staff, _1));
    ros::Subscriber obj_sub = nh.subscribe<geometry_msgs::PoseStamped>("/obj_debug",10,boost::bind(&ObjSub::Obj_posSub, &fsm.obj_staff, _1));
    ros::Subscriber qr_sub = nh.subscribe<std_msgs::String>("/qr_code_info",100, boost::bind(&QrSub::Qr_cb, &fsm.qr_num, _1));
    ros::Subscriber traj_judge_staff = nh.subscribe<std_msgs::UInt8>("/traj_start",10, boost::bind(&TrajSub::Traj_Sub, &fsm.traj_judge_staff, _1));
    ros::Subscriber dynamic_sub = nh.subscribe<std_msgs::Bool>("/delivery/drop_command",10, boost::bind(&DynamicSub::Dynamic_CB, &fsm.dynamic_staff, _1));

    fsm.position_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    fsm.cross_pub = nh.advertise<geometry_msgs::PoseStamped>("/planner2/goal",10);

    fsm.pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_pub",10);
    fsm.arming_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    fsm.circle_pub = nh.advertise<std_msgs::Int8>("/circle_detection_control",10);
    fsm.classify_debug_pub = nh.advertise<mission_node::class_pub>("/yolo/classify_debug",10);
    fsm.judge_start_pub = nh.advertise<std_msgs::Int8>("/judge_start",10);
    // 发布起飞/降落指令
    fsm.takeoff_land_pub = nh.advertise<std_msgs::UInt8>("/Takeoff_judge",10);
    fsm.obj_pub = nh.advertise<mavros_msgs::PositionTarget>("/obj_node",10);
    ros::Duration(2.0).sleep();  // 等待一段时间
    while (ros::ok())
    {
        fsm.process();  // 调用 MissionFSM 的 process 方法
        ros::spinOnce();  // 处理回调
        rate.sleep();     // 控制循环频率
    }

    return 0;
}