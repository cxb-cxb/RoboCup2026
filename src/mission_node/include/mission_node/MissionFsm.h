#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include "input.h"
#include <quadrotor_msgs/TakeoffLand.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/CommandLong.h>  // 确保包含必要的消息类型
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <mission_node/class_pub.h>
#include <queue>

#ifndef _MISSIONFSM_H
#define _MISSIONFSM_H

static uint8_t Small_drop = 'C';
static uint8_t small_drop = 'U';
static uint8_t Big_drop = 'P';
static float x_threshold = 0.1;
static float y_threshold = 0.1;
static float last_error_x = 0;
static float last_error_y = 0;
static float predict_y=0;
static float predict_x=0;


class MissionFSM
{
    public:
        MissionFSM();
        void process(); 
        void pose_pub(const std::vector<geometry_msgs::PoseStamped>& target_points,int flag);
        bool first_pub(const std::vector<geometry_msgs::PoseStamped>& points);
        void enableEmergency();
        // void pid_control();
        void Ser_pub(uint8_t num);
        void computeAdjustment(double u, double v, double depth, const geometry_msgs::Quaternion& ros_quat, 
            double& delta_x, double& delta_y, double& delta_z);
        double quaternionToYaw(const geometry_msgs::Quaternion& quat);
        double getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
            double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr);
        // float constrain(float value, float min_val, float max_val);

        double getYawDifference(const geometry_msgs::Quaternion& quat1, 
                            const geometry_msgs::Quaternion& quat2);

        double getAbsYawDifference(const geometry_msgs::Quaternion& quat1, 
                                const geometry_msgs::Quaternion& quat2);

        double getYawDifferenceDegrees(const geometry_msgs::Quaternion& quat1, 
                                    const geometry_msgs::Quaternion& quat2);
        // 新增：数据采集相关公共函数
        void startDataCollection();                    // 开始数据采集
        void stopDataCollection();                     // 停止数据采集
        void collectFlightData();                      // 在飞行过程中采集数据
        bool calculateClassBasedMedianAdjustment(double& median_dx, double& median_dy, std::string& dominant_class); // 计算中位数调整值
        void calculateRandomMedianTarget();
        bool checkWithCount(const std::string& class_name);
        double Point_ToYAW(geometry_msgs::PoseStamped point_1,geometry_msgs::PoseStamped point_2);
        void Point_ToYAW_WithQuaternion(geometry_msgs::PoseStamped point_1, 
                                   geometry_msgs::PoseStamped point_2,
                                   geometry_msgs::Quaternion& quaternion);
        double calculateMovingDirection(const std::vector<geometry_msgs::Point>& positions);
        ros::Publisher position_pub;
        ros::Publisher takeoff_land_pub;
        ros::Publisher pos_pub;
        ros::Publisher classify_debug_pub;
        ros::Publisher judge_start_pub;
        ros::Publisher circle_pub;
        ros::Publisher obj_pub;
        ros::ServiceClient arming_client; 
        ros::Publisher cross_pub;  

        StateSub state_mission;
        TrajSub traj_judge_staff;
        CircleSub circle_staff;
        PoseSub pose_data;
        DynamicSub dynamic_staff;
        QrSub qr_num;
        ImageSub image_staff;
        ClassifySub class_staff;
        ObjSub obj_staff;
        DirectionSub direct_flag;
        mission_node::class_pub classify_debug;
        //mavros_msgs::PositionTarget setpoint_raw;
        //mavros_msgs::SetMode offb_set_mode;
        //mavros_msgs::CommandBool arm_cmd;
        std::vector<geometry_msgs::PoseStamped> target_points;
        std::vector<geometry_msgs::PoseStamped> test_points;
        std_msgs::String current_class;
        std_msgs::Int8 judge_start;
        std_msgs::Int8 circle_control;
        std::vector<geometry_msgs::PoseStamped> first_points;
        geometry_msgs::PoseStamped detect_point;
        geometry_msgs::PoseStamped decide_right;
        geometry_msgs::PoseStamped decide_left;
        geometry_msgs::PoseStamped land_right;
        geometry_msgs::PoseStamped land_left;
        geometry_msgs::PoseStamped land_point;
        geometry_msgs::PoseStamped Debug_point;
        geometry_msgs::PoseStamped change_yaw;
        geometry_msgs::PoseStamped cross_01;
        geometry_msgs::PoseStamped cross_circle;
        geometry_msgs::PoseStamped hight_point;//用于投货完成后回到1.2米的高度
        geometry_msgs::PoseStamped cross_ring;
        geometry_msgs::PoseStamped decide_land;
        geometry_msgs::PoseStamped obj_position;
        geometry_msgs::PoseStamped dynamic_position;
        geometry_msgs::PoseStamped heighting_point;
        geometry_msgs::PoseStamped finish_Point;
        geometry_msgs::PoseStamped dynamic_point;
        geometry_msgs::PoseStamped obj_point;
        geometry_msgs::PoseStamped cross_point;
        geometry_msgs::PoseStamped cross_point_02;
        geometry_msgs::PoseStamped cross_point_03;
        geometry_msgs::PoseStamped cross_land_point;
        // geometry_msgs::PoseStamped change_yaw_point;
        mavros_msgs::PositionTarget Obj_vel; 
        geometry_msgs::PoseStamped drop_finish_point;
        geometry_msgs::PoseStamped decide_track;
        geometry_msgs::PoseStamped high_drop;
        //两个端点确定靶标的方向
        geometry_msgs::PoseStamped duan_point_1;
        geometry_msgs::PoseStamped duan_point_2;


        //random_add value
        // 添加这些成员变量
        std::vector<std::pair<double, double>> random_positions;  // 存储random目标位置
        const size_t RANDOM_SAMPLE_THRESHOLD = 8;  // 达到20个样本后使用中位数
        bool use_random_median = false;  // 标记是否使用random中位数作为目标
        geometry_msgs::PoseStamped random_median_target;  // 存储计算出的中位数目标点

        struct DeltaPair {
            double delta_x;
            double delta_y;
            ros::Time timestamp;
            DeltaPair(double x, double y, ros::Time t) : delta_x(x), delta_y(y), timestamp(t) {}
        };
        // 新增：飞行数据样本结构体
        struct FlightDataSample {
            double tar_x;                          // x目标位置
            double tar_y;                          // y目标位置
            std_msgs::String cur_class;
            ros::Time timestamp;                     // 时间戳
            geometry_msgs::Point drone_position;     // 无人机位置（可选，用于调试）
        };
        std::vector<DeltaPair> delta_vector;
        enum class DroneState {
            INIT,
            TAKEOFF,
            DECIDE_TRACK,
            CROSS_RING,
            DECIDE_LAND,
            TRACKING_WAYPOINT,
            DROPING,
            CROSS_LAND,
            HIGHING,
            HIGH_DROP,
            FINISH_DROP,
            TO_RANDOM,
            RANDDOM_DROPING,
            DECIDE_CROSS,
            DECIDE_CROSS02,
            DECIDE_CROSS03,
            JUDGE_CROSS,
            JUDGE_CROSS02,
            JUDGE_CROSS03,
            DECIDE_DYNAMIC,
            DECIDE_DYNAMIC_02,
            YAW_FINISH,
            DYNAMIC_DROP,
            CHANGE_YAW,
            // PREPARE_TUNNEL,
            FINISH_Dynamic,
            DEBUG02,
            LAND,
            DEBUG,
            FINISH
        };
        enum class DyDropState {
            SEARCHING,    // 搜索目标
            TRACKING,     // 跟踪目标
            HIGHTING,     // 投放货物
            FINISHED      // 任务完成
        };

    private:
        DroneState current_state;
        DyDropState current_drone_state;
        ros::Rate rate;
        int mission_num;
        bool droping_flag;
        bool droping_second;
        bool yaw_judge;
        bool flag_no_object;
        bool cross_judge;
        bool second_adjust;
        int droping_i;
        int goods_num;
        bool last_judge;
        bool cargo_dropped;
        bool second_judge;
        bool trj_contral;
        bool ego_contral;
        //跟踪目标的pid
        float error_x;
        float error_y;
        float linear_x_p;
        float linear_x_d;
        float linear_y_p;
        float linear_y_d;
        float vx;
        float vy;
        float predict_x;
        float predict_y;
        float last_target_x;
        float last_target_y;
        std::queue<uint8_t> Drop_queue;
        std::set<std::string> dropped_classes;
        // std::queue<>
        // 新增：数据采集相关成员变量
        bool is_collecting_data;                       // 是否正在采集数据的标志
        std::vector<FlightDataSample> flight_data_samples; // 飞行过程中采集的数据样本
        ros::Time data_collection_start_time;          // 数据采集开始时间

        //移动靶标方向计算
        bool is_sampling_yaw;
        std::vector<geometry_msgs::Point> target_trajectory_samples;
        ros::Time yaw_sampling_start_time;
};

#endif