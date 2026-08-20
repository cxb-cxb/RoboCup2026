#include "input.h"
#include "MissionFsm.h"
// #include "input.h"
#include <quadrotor_msgs/TakeoffLand.h>
#include "cmath"
#include <vector>
#include <algorithm>
#include <tf/tf.h>  // 或者 #include <tf2/LinearMath/Quaternion.h>
const size_t REQUIRED_DATA_COUNT = 20; // 需要20个数据对

//调整量
double delta_x, delta_y, delta_z;
geometry_msgs::Quaternion debug_quat;
//微调位置
geometry_msgs::PoseStamped Adjust_point;


//构造函数
MissionFSM::MissionFSM() : rate(20.0) 
{
    ros::NodeHandle private_nh("~");//声明一个私有的句柄，用来接收参数服务器的参数
    GetParameters(private_nh);

    search_planner_.configure(
        parameters_.search_points_x, parameters_.search_points_y,
        parameters_.search_height, parameters_.max_search_passes);
    remaining_classes_ = {"random", "tank", "car", "bridge", "tent", "bunker"};
    completed_drops_ = 0;
    search_adjust_phase_ = 0;
    search_goal_sent_ = false;
    active_search_class_.clear();

    // 其他初始化
    current_state = DroneState::INIT;
    // current_state = DroneState::DECIDE_CROSS;
    current_drone_state = DyDropState::TRACKING;
    Drop_queue.push('C');
    Drop_queue.push('U');
    Drop_queue.push('P');
    mission_num = 0;
    // linear_x_d = linear_y_d = 0.05;
    // 在构造函数末尾添加
    is_sampling_yaw = false;
    target_trajectory_samples.clear();


    droping_flag = true ;
    droping_second = false;
    second_adjust = true;
    yaw_judge = false;
    cross_judge = true;
    cargo_dropped = false;
    drop_random = false;
    ego_contral = true;
    droping_i = 300; 
    goods_num = 2;
    last_target_x=0;
    last_target_y=0;
    //random init
    random_positions_1.clear();
    random_positions_2.clear();  // 新增:初始化第二个点集
    use_random_median = false;

    is_collecting_data = false;
    flight_data_samples.clear();

    //根据两个端点确定移动靶标的朝向

    duan_point_1.pose.position.x = -1.93;
    duan_point_1.pose.position.y = 6.7;


    duan_point_2.pose.position.x = -0.13;
    duan_point_2.pose.position.y = 4.93;


    cross_point.header.frame_id = "camera_init";
    // cross_point.pose.position.x = 9.0;
    // cross_point.pose.position.y = 1.95;
    // cross_point.pose.position.z = 0.6;
    cross_point.pose.position.x = -3.2;//9.0;
    cross_point.pose.position.y = 8.0;//2.0;
    cross_point.pose.position.z = 0.6;
    cross_point.pose.orientation.x = 0;
    cross_point.pose.orientation.y = 0;
    cross_point.pose.orientation.z = 0;
    cross_point.pose.orientation.w = 1;
    // dynamic_position.pose.position.x = 2.2;
    // dynamic_position.pose.position.y = 0.0;
    // dynamic_position.pose.position.z = 1.0;
    // dynamic_position.pose.orientation.x = 0;
    // dynamic_position.pose.orientation.y = 0;
    // dynamic_position.pose.orientation.z = 0;
    // dynamic_position.pose.orientation.w = 1;
    //current_state = DroneState::DROPING;

	//一定要记得控制器改 起飞高度
    cross_point_02.header.frame_id = "camera_init";
    // cross_point_02.pose.position.x = 9.0;
    // cross_point_02.pose.position.y = -0.65;
    // cross_point_02.pose.position.z = 0.6;
    cross_point_02.pose.position.x = 1.76;//9.1;
    cross_point_02.pose.position.y = 8.00;//-0.48;
    cross_point_02.pose.position.z = 0.6;
    cross_point_02.pose.orientation.x = 0;
    cross_point_02.pose.orientation.y = 0;
    cross_point_02.pose.orientation.z = 0;
    cross_point_02.pose.orientation.w = 1;

    //
    cross_point_03.header.frame_id = "camera_init";
    // cross_point_02.pose.position.x = 9.0;
    // cross_point_02.pose.position.y = -0.65;
    // cross_point_02.pose.position.z = 0.6;
    cross_point_03.pose.position.x = 1.55;
    cross_point_03.pose.position.y = -3.6;
    cross_point_03.pose.position.z = 0.6;
    cross_point_03.pose.orientation.x = 0;
    cross_point_03.pose.orientation.y = 0;
    cross_point_03.pose.orientation.z = 0;
    cross_point_03.pose.orientation.w = 1;
    //由特殊把到目标点的避障点
    geometry_msgs::PoseStamped first_1;
    first_1.pose.position.x = 4.5;
    first_1.pose.position.y = 0.5;
    first_1.pose.position.z = 1.0;
        //yaw角设置
    first_1.pose.orientation.x = 0;
    first_1.pose.orientation.y = 0;
    first_1.pose.orientation.z = 0;
    first_1.pose.orientation.w = 1;
    first_points.push_back(first_1);
    // 第二个目标点

    geometry_msgs::PoseStamped first_2;
    first_2.pose.position.x = 4.4;
    first_2.pose.position.y = -3.1;
    first_2.pose.position.z = 1.0;
    first_2.pose.orientation.x = 0;
    first_2.pose.orientation.y = 0;
    first_2.pose.orientation.z = 0;
    first_2.pose.orientation.w = 1;
    first_points.push_back(first_2);

    geometry_msgs::PoseStamped first_3;
    first_3.pose.position.x = 1.9;
    first_3.pose.position.y = -1.6;
    first_3.pose.position.z = 1.0;
    first_3.pose.orientation.x = 0;
    first_3.pose.orientation.y = 0;
    first_3.pose.orientation.z = 0;
    first_3.pose.orientation.w = 1;
    first_points.push_back(first_3);

    geometry_msgs::PoseStamped first_4;
    first_4.pose.position.x = 0;
    first_4.pose.position.y = 0;
    first_4.pose.position.z = 1.0;
    first_4.pose.orientation.x = 0;
    first_4.pose.orientation.y = 0;
    first_4.pose.orientation.z = 0;
    first_4.pose.orientation.w = 1;
    first_points.push_back(first_4);
    // //动态靶标轨迹中心点
    // //降落
    // dynamic_point.pose.position.x = 6.55;
    // dynamic_point.pose.position.y = 1.3;
    // dynamic_point.pose.position.z = 1.0;
    
}

void MissionFSM::process() 
{
    
    static std_msgs::UInt8 takeoff_land_msg;
    //目标点声明
    //状态机的实现
    switch(current_state) {
        case DroneState::INIT:
            ROS_INFO("Initializing...");
            current_state = DroneState::TAKEOFF ;//TAKEOFF
            break; 
        case DroneState::TAKEOFF:
            while (ros::ok() && !state_mission.state_code.connected)
            {
                    ROS_INFO("connecting----");
                    ros::spinOnce();
                    rate.sleep();
            }
            ROS_INFO("TAKE OFF");
            
            takeoff_land_msg.data = 1;  // 设置命令为 1 , 起飞
            takeoff_land_pub.publish(takeoff_land_msg);
            if(std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.1 && traj_judge_staff.traj_sub.data == 1)
            {
                ROS_INFO("success");
                //完整流程
                ros::Duration(2.0).sleep();
                current_state = DroneState::DECIDE_TRACK;
                ROS_INFO("TAKE SUCCESS");
            }
            break; 
        case DroneState::DECIDE_TRACK:
            //ego-planner的调试
            decide_track.pose.position.x = 0;
            decide_track.pose.position.y = 0;
            decide_track.pose.position.z = 1.0;
            decide_track.pose.orientation.x = 0;
            decide_track.pose.orientation.y = 0;
            decide_track.pose.orientation.z = 0;
            decide_track.pose.orientation.w = 1;
            pos_pub.publish(decide_track);
            if (std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.05 && std::abs(pose_data.pose_local.pose.position.x - 0.1) <0.05)
            {
                //逻辑改变，将TRACK_WAYPOINT转换成搜索
                current_state = DroneState::GENERATE_SEARCH;
                ROS_INFO("OK");
            }
            break;
        case DroneState::GENERATE_SEARCH:
            search_planner_.generate();
            startDataCollection();
            if (search_planner_.empty() ||
                !search_planner_.resumeNearest(pose_data.pose_local.pose.position)) {
                ROS_ERROR("Coverage search produced no waypoints");
                current_state = DroneState::SEARCH_FINISHED;
            } else {
                search_goal_sent_ = false;
                current_state = DroneState::SEARCHING;
                ROS_INFO("Coverage search started with pass %d",
                         search_planner_.passCount());
            }
            break;

        case DroneState::SEARCHING:
            recordSearchDetection();
            if (current_state != DroneState::SEARCHING) break;

            if (search_planner_.hasCurrentWaypoint()) {
                const geometry_msgs::PoseStamped& waypoint =
                    search_planner_.currentWaypoint();
                if (!search_goal_sent_) {
                    position_pub.publish(waypoint);
                    search_goal_sent_ = true;
                }
                pos_pub.publish(waypoint);
                if (getLengthBetweenPoints(pose_data.pose_local.pose.position,
                                           waypoint.pose.position) < 0.15) {
                    search_planner_.markCurrentVisited();
                    search_goal_sent_ = false;
                }
                break;
            }

            if (completed_drops_ < 3 && selectCachedLowPriorityTarget()) break;
            if (search_planner_.startNextPass(pose_data.pose_local.pose.position)) {
                search_goal_sent_ = false;
                ROS_INFO("Starting coverage pass %d", search_planner_.passCount());
            } else {
                current_state = DroneState::SEARCH_FINISHED;
            }
            break;

        case DroneState::APPROACH_DETECTED_TARGET:
            recordSearchDetection();
            if (!search_goal_sent_) {
                position_pub.publish(active_search_target_);
                search_goal_sent_ = true;
            }
            pos_pub.publish(active_search_target_);
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,
                                       active_search_target_.pose.position) < 0.15) {
                search_adjust_phase_ = 0;
                search_phase_started_ = ros::Time::now();
                search_goal_sent_ = false;
                current_state = DroneState::SEARCH_TARGET_DROP;
            }
            break;

        case DroneState::SEARCH_TARGET_DROP:
        {
            const bool target_visible =
                image_staff.image_data.detected_class == active_search_class_ &&
                image_staff.image_data.cx != 0 && image_staff.image_data.cy != 0;

            if (search_adjust_phase_ == 0) {
                if (!target_visible) {
                    if ((ros::Time::now() - search_phase_started_).toSec() > 5.0)
                        cancelSearchTarget();
                    break;
                }
                computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                                  pose_data.pose_local.pose.position.z,
                                  pose_data.pose_local.pose.orientation,
                                  delta_x, delta_y, delta_z);
                Adjust_point.header.frame_id = "camera_init";
                Adjust_point.pose.position.x = pose_data.pose_local.pose.position.x + delta_x;
                Adjust_point.pose.position.y = pose_data.pose_local.pose.position.y + delta_y;
                Adjust_point.pose.position.z = parameters_.search_height;
                Adjust_point.pose.orientation.x = 0.0;
                Adjust_point.pose.orientation.y = 0.0;
                Adjust_point.pose.orientation.z = 0.0;
                Adjust_point.pose.orientation.w = 1.0;
                position_pub.publish(Adjust_point);
                pos_pub.publish(Adjust_point);
                search_adjust_phase_ = 1;
                search_phase_started_ = ros::Time::now();
                break;
            }

            if (search_adjust_phase_ == 1) {
                pos_pub.publish(Adjust_point);
                if (getLengthBetweenPoints(pose_data.pose_local.pose.position,
                                           Adjust_point.pose.position) >= 0.15)
                    break;
                if (!target_visible) {
                    if ((ros::Time::now() - search_phase_started_).toSec() > 5.0)
                        cancelSearchTarget();
                    break;
                }
                if (Drop_queue.empty()) {
                    current_state = DroneState::SEARCH_FINISHED;
                    break;
                }
                computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                                  pose_data.pose_local.pose.position.z,
                                  pose_data.pose_local.pose.orientation,
                                  delta_x, delta_y, delta_z);
                double cargo_offset_x = 0.0;
                if (Drop_queue.front() == 'C') cargo_offset_x = 0.2;
                else if (Drop_queue.front() == 'U') cargo_offset_x = -0.2;
                Adjust_point.pose.position.x =
                    pose_data.pose_local.pose.position.x + delta_x + cargo_offset_x;
                Adjust_point.pose.position.y =
                    pose_data.pose_local.pose.position.y + delta_y;
                Adjust_point.pose.position.z = 0.2;
                position_pub.publish(Adjust_point);
                pos_pub.publish(Adjust_point);
                search_adjust_phase_ = 2;
                break;
            }

            pos_pub.publish(Adjust_point);
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,
                                       Adjust_point.pose.position) < 0.05) {
                completeSearchDrop();
            }
            break;
        }

        case DroneState::RESUME_SEARCH:
            hight_point.header.frame_id = "camera_init";
            hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
            hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
            hight_point.pose.position.z = parameters_.search_height;
            hight_point.pose.orientation.x = 0.0;
            hight_point.pose.orientation.y = 0.0;
            hight_point.pose.orientation.z = 0.0;
            hight_point.pose.orientation.w = 1.0;
            if (!search_goal_sent_) {
                position_pub.publish(hight_point);
                search_goal_sent_ = true;
            }
            pos_pub.publish(hight_point);
            if (std::abs(pose_data.pose_local.pose.position.z -
                         parameters_.search_height) < 0.1) {
                search_goal_sent_ = false;
                if (search_planner_.resumeNearest(pose_data.pose_local.pose.position)) {
                    current_state = DroneState::SEARCHING;
                } else if (completed_drops_ < 3 && selectCachedLowPriorityTarget()) {
                    break;
                } else if (search_planner_.startNextPass(
                               pose_data.pose_local.pose.position)) {
                    current_state = DroneState::SEARCHING;
                } else {
                    current_state = DroneState::SEARCH_FINISHED;
                }
            }
            break;

        case DroneState::SEARCH_FINISHED:
            if (completed_drops_ >= 3) {
                ROS_INFO("Coverage delivery completed with three drops");
            } else {
                ROS_ERROR("Coverage search ended after %d passes with only %d drops",
                          search_planner_.passCount(), completed_drops_);
            }
            finishSearchMission();
            break;

        case DroneState::HIGHING:
            Debug_point.pose.position.x = pose_data.pose_local.pose.position.x;
            Debug_point.pose.position.y = pose_data.pose_local.pose.position.y;
            Debug_point.pose.position.z = 1.0;
            //yaw角设置
            Debug_point.pose.orientation.x = 0;
            Debug_point.pose.orientation.y = 0;
            Debug_point.pose.orientation.z = 0;
            Debug_point.pose.orientation.w = 1;
            pos_pub.publish(Debug_point);
            ROS_INFO("--high--");
            if (std::abs(pose_data.pose_local.pose.position.z - 1.0 < 0.05) && getAbsYawDifference(pose_data.pose_local.pose.orientation,Debug_point.pose.orientation) < 0.1)
            {
                ros::Duration(1.0).sleep();
                ros::spinOnce();
                current_state = DroneState::FINISH_Dynamic;
            }
            break;
        case DroneState::TRACKING_WAYPOINT:
            ROS_INFO(" TO TARGET POINT ");
            collectFlightData();

            // judge_start_pub.publish();
            if(mission_num < 4) //&& goods_num!=0)
            {
                ROS_INFO("CONINTIUTE ");
                if(std::abs(pose_data.pose_local.pose.position.z - 1.0) < 0.15 || std::abs(pose_data.pose_local.pose.position.z - 1.2) < 0.05)
                {
                    ROS_INFO("----high----");
                    pose_pub(target_points,mission_num);
                    printf("num:%d \n",mission_num);
                    // judge_start.data =0;
                    // judge_start_pub.publish(judge_start);
                }
            }
            if(mission_num == 4)
            {
                if(std::abs(pose_data.pose_local.pose.position.z - 1.0) < 0.15)
                {
                    ROS_INFO("----high----");
                    // pose_pub(target_points,mission_num);
                    // printf("num:%d \n",mission_num);
                    // judge_start.data =0;
                    current_state = DroneState::FINISH_DROP ;//准备降落或者去穿隧道

                    // judge_start_pub.publish(judge_start);
                    ROS_INFO("finish");
                }

            }
            break; 
        case DroneState::FINISH_DROP:

            drop_finish_point.pose.position.x = -3.27;
            drop_finish_point.pose.position.y = 6.57;
            drop_finish_point.pose.position.z = 1.2;
            drop_finish_point.pose.orientation.x = 0;
            drop_finish_point.pose.orientation.y = 0;
            drop_finish_point.pose.orientation.z = 0;
            drop_finish_point.pose.orientation.w = 1;
            if(ego_contral)
            {
                // startDataCollection();
                collectFlightData();
                position_pub.publish(drop_finish_point);
                ego_contral = false;
                ROS_INFO("--sending--");
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(drop_finish_point);
            if (std::abs(pose_data.pose_local.pose.position.x - drop_finish_point.pose.position.x) <0.05 && std::abs(pose_data.pose_local.pose.position.z - drop_finish_point.pose.position.z)<0.05 && std::abs(pose_data.pose_local.pose.position.y - drop_finish_point.pose.position.y)<0.05)
            {
                if (use_random_median && !drop_random) 
                {
                    ROS_INFO("DECIDE TO_RANDOM - Random target detected");
                    current_state = DroneState::TO_RANDOM;
                    ego_contral = true;
                }
                else
                {
                    ROS_INFO("DECIDE DYNAMIC");
                    // current_state = DroneState::DECIDE_DYNAMIC;
                    current_state = DroneState::FINAL_MISSION;
                }
            }
            
            break; 
        case DroneState::FINAL_MISSION:
            ROS_INFO("TO_RANDOM: Flying to BUNKER target");
            
            // 确保random_median_target已经设置好高度和姿态
            // final_mission.pose.position.x = -1.0;
            // final_mission.pose.position.y = 2.7;
            // final_mission.pose.position.z = 1.0;

            // final_mission.pose.position.x = -2.5;
            // final_mission.pose.position.y = 4.7;
            // final_mission.pose.position.z = 1.0;

            final_mission.pose.position.x = 2.72;
            final_mission.pose.position.y = 1.08;
            final_mission.pose.position.z = 1.0;

            // final_mission.pose.position.x = -3.2;
            // final_mission.pose.position.y = 1.7;
            // final_mission.pose.position.z = 1.0;


            final_mission.pose.orientation.x = 0;
            final_mission.pose.orientation.y = 0;
            final_mission.pose.orientation.z = 0;
            final_mission.pose.orientation.w = 1;
            
            // 第一次进入状态时发送给ego-planner
            if(ego_contral)
            {
                position_pub.publish(final_mission);
                startDataCollection();
                ego_contral = false;
                ROS_INFO("--sending random target to ego-planner--");
                ros::Duration(1.0).sleep();
            }
            position_pub.publish(final_mission);
            pos_pub.publish(final_mission);
            // 持续发布位置指令
            // pos_pub.publish(final_mission);
            
            // // 显示当前位置误差
            // ROS_INFO_THROTTLE(1.0, "Distance to random target: %.3f", 
            //                 getLengthBetweenPoints(pose_data.pose_local.pose.position, 
            //                                         random_median_target.pose.position));
            
            // 判断是否到达random目标点
            if(std::abs(pose_data.pose_local.pose.position.x - final_mission.pose.position.x) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.y - final_mission.pose.position.y) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.z - final_mission.pose.position.z) < 0.1)
            {
                ROS_INFO("Arrived at random target, preparing to drop");

                // 重置标志位
                droping_flag = true;
                droping_second = true;
                second_adjust = true;
                ego_contral = true;
                
                // 切换到投货状态
                current_state = DroneState::FINAL_DROPING;
                ROS_INFO("Switched to DROPING state for random target");
            }
            break;
        case DroneState::FINAL_DROPING:
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            ROS_INFO("DROP");
            // judge_start.data =1;
            // judge_start_pub.publish(judge_start);
            if(!image_staff.image_data.detected_class.empty()) //|| !class_staff.class_data.classify_class.empty())
            {
                if(droping_flag)
                {
                    // 停止数据采集
                    // 停止数据采集
                    stopDataCollection();
                    // 使用基于类别统计的中位数进行微调
                    double median_x, median_y;
                    std::string target_class;            
                    if(calculateClassBasedMedianAdjustment(median_x, median_y, target_class)) 
                    {
                        // 使用统计得出的主要类别的中位数位置进行微调
                        Adjust_point.pose.position.x = median_x;
                        Adjust_point.pose.position.y = median_y;
                        Adjust_point.pose.position.z = 1.0;
                        Adjust_point.pose.orientation.x = 0;
                        Adjust_point.pose.orientation.y = 0;
                        Adjust_point.pose.orientation.z = 0;
                        Adjust_point.pose.orientation.w = 1;
                
                        // 更新当前类别为主要检测到的类别
                        current_class.data = target_class;
                        printf("Using filtered median for class '%s' - x:%.3f, y:%.3f\n remain class's count is %ld\n", 
                        target_class.c_str(), median_x, median_y,dropped_classes.size());
                       
                        ROS_INFO("Applied median adjustment based on current detection: %s", target_class.c_str()); image_staff.image_data.detected_class;                

                    } 
                    else 
                    {
                        // 如果基于类别的统计失败，使用当前检测进行微调
                        computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                                pose_data.pose_local.pose.position.z,
                                pose_data.pose_local.pose.orientation,
                                delta_x, delta_y, delta_z);
                                Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x;
                                Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y;
                                Adjust_point.pose.position.z = 1.0;
                                Adjust_point.pose.orientation.x = 0;
                                Adjust_point.pose.orientation.y = 0;
                                Adjust_point.pose.orientation.z = 0;
                                Adjust_point.pose.orientation.w = 1;
                            
                        current_class.data = image_staff.image_data.detected_class;
                        printf("Using current detection fallback - x:%.3f, y:%.3f\n", 
                                delta_x, delta_y);
                    }
                    droping_flag = false;
                    printf("class:%s\n", current_class.data.c_str());
                    pos_pub.publish(Adjust_point);
                    droping_second = true;
                }
                // image_staff.image_data.detected_class=  class_staff.classfiy_data.data;
                if( droping_second && getLengthBetweenPoints(pose_data.pose_local.pose.position,Adjust_point.pose.position)<0.15)
                {
                    printf("success\n");
                    ros::Duration(1.0).sleep();
                    ros::spinOnce();
                    if(image_staff.image_data.detected_class == "bunker")
                    {
                        //根据第一个进行调整 :P
                        computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                        Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x ;
                        Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y +0.3;
                        printf("x:%f\n",Adjust_point.pose.position.x);
                        printf("y:%f\n",Adjust_point.pose.position.y);
                        Adjust_point.pose.position.z = 0.2;
                        Adjust_point.pose.orientation.x = 0;
                        Adjust_point.pose.orientation.y = 0;
                        Adjust_point.pose.orientation.z = 0;
                        Adjust_point.pose.orientation.w = 1;
                        ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                        // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x + 0.2;
                        // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y ;
                        // //yaw角设置
                        // Adjust_point.pose.orientation.x = 0;
                        // Adjust_point.pose.orientation.y = 0;
                        // Adjust_point.pose.orientation.z = 0;
                        // Adjust_point.pose.orientation.w = 1;
                        pos_pub.publish(Adjust_point);
                        second_adjust = false;
                        droping_second = false;
                        // ROS_INFO("3");
                    }
                }
                if(std::abs(pose_data.pose_local.pose.position.z - 0.2) < 0.05 && std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x) <0.03 && std::abs(pose_data.pose_local.pose.position.y - Adjust_point.pose.position.y)<0.03)
                { 
                     Ser_pub('P');
                    Drop_queue.pop();
                    // current_class.data = "";
                    ROS_INFO("Droping Finsh ");
                    droping_flag =  true;
                    droping_second = true;
                    second_adjust = true;
                    ros::Duration(1.5).sleep();
                    ros::spinOnce();
                    current_state = DroneState::HIGHING;
                    hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                    hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                    hight_point.pose.position.z = 1.0;
                    hight_point.pose.orientation.x = 0;
                    hight_point.pose.orientation.y = 0;
                    hight_point.pose.orientation.z = 0;
                    hight_point.pose.orientation.w = 1;
                    pos_pub.publish(hight_point);
                    // startDataCollection();
                }                     
            }
            else
            {
                //ROS_INFO("debug");const
                if(droping_i>0)
                {
                    droping_i--;
                    printf("num: %d",droping_i);
                }
                if(droping_i <=0)
                {
                    current_state = DroneState::LAND;
                    ROS_INFO("FAILED");
                    droping_i = 200;
                }
            }
            break;


        case DroneState::TO_RANDOM:
            ROS_INFO("TO_RANDOM: Flying to random target");
            
            // 确保random_median_target已经设置好高度和姿态
            random_median_target.pose.position.z = 1.0;
            random_median_target.pose.orientation.x = 0;
            random_median_target.pose.orientation.y = 0;
            random_median_target.pose.orientation.z = 0;
            random_median_target.pose.orientation.w = 1;
            
            // 第一次进入状态时发送给ego-planner
            if(ego_contral)
            {
                position_pub.publish(random_median_target);
                startDataCollection();
                ego_contral = false;
                ROS_INFO("--sending random target to ego-planner--");
                ros::Duration(1.0).sleep();
            }
            
            // 持续发布位置指令
            pos_pub.publish(random_median_target);
            
            // 显示当前位置误差
            ROS_INFO_THROTTLE(1.0, "Distance to random target: %.3f", 
                            getLengthBetweenPoints(pose_data.pose_local.pose.position, 
                                                    random_median_target.pose.position));
            
            // 判断是否到达random目标点
            if(std::abs(pose_data.pose_local.pose.position.x - random_median_target.pose.position.x) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.y - random_median_target.pose.position.y) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.z - random_median_target.pose.position.z) < 0.1)
            {
                ROS_INFO("Arrived at random target, preparing to drop");
                
                // // 标记random类别为当前目标

                // 重置标志位
                use_random_median = false;
                droping_flag = true;
                droping_second = true;
                second_adjust = true;
                ego_contral = true;
                
                // 切换到投货状态
                current_state = DroneState::RANDDOM_DROPING;
                ROS_INFO("Switched to DROPING state for random target");
            }
            break;
        case DroneState::RANDDOM_DROPING:
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            ROS_INFO("DROP");
            // judge_start.data =1;
            // judge_start_pub.publish(judge_start);
            if(!image_staff.image_data.detected_class.empty()) //|| !class_staff.class_data.classify_class.empty())
            {
                if(droping_flag)
                {
                    // 停止数据采集
                    // 停止数据采集
                    stopDataCollection();
                    // 使用基于类别统计的中位数进行微调
                    double median_x, median_y;
                    std::string target_class;            
                    if(calculateClassBasedMedianAdjustment(median_x, median_y, target_class)) 
                    {
                        // 使用统计得出的主要类别的中位数位置进行微调
                        Adjust_point.pose.position.x = median_x;
                        Adjust_point.pose.position.y = median_y;
                        Adjust_point.pose.position.z = 1.0;
                        Adjust_point.pose.orientation.x = 0;
                        Adjust_point.pose.orientation.y = 0;
                        Adjust_point.pose.orientation.z = 0;
                        Adjust_point.pose.orientation.w = 1;
                
                        // 更新当前类别为主要检测到的类别
                        current_class.data = target_class;
                        if (target_class == "random")
                        {
                            dropped_classes.erase("random");
                        }
                        else if (target_class == "tank")
                        {
                            dropped_classes.erase("tank");
                        }
                        printf("Using filtered median for class '%s' - x:%.3f, y:%.3f\n remain class's count is %ld\n", 
                        target_class.c_str(), median_x, median_y,dropped_classes.size());
                       
                        ROS_INFO("Applied median adjustment based on current detection: %s", target_class.c_str()); image_staff.image_data.detected_class;                

                    } 
                    else 
                    {
                        // 如果基于类别的统计失败，使用当前检测进行微调
                        computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                                pose_data.pose_local.pose.position.z,
                                pose_data.pose_local.pose.orientation,
                                delta_x, delta_y, delta_z);
                                Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x;
                                Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y;
                                Adjust_point.pose.position.z = 1.0;
                                Adjust_point.pose.orientation.x = 0;
                                Adjust_point.pose.orientation.y = 0;
                                Adjust_point.pose.orientation.z = 0;
                                Adjust_point.pose.orientation.w = 1;
                            
                        current_class.data = image_staff.image_data.detected_class;
                        printf("Using current detection fallback - x:%.3f, y:%.3f\n", 
                                delta_x, delta_y);
                    }
                    droping_flag = false;
                    printf("class:%s\n", current_class.data.c_str());
                    pos_pub.publish(Adjust_point);
                    droping_second = true;
                }
                // image_staff.image_data.detected_class=  class_staff.classfiy_data.data;
                if( droping_second && getLengthBetweenPoints(pose_data.pose_local.pose.position,Adjust_point.pose.position)<0.15)
                {
                    printf("success\n");
                    ros::Duration(1.0).sleep();
                    ros::spinOnce();
                    if(image_staff.image_data.detected_class == "random" || image_staff.image_data.detected_class == "tank")
                    {
                        //根据第一个进行调整 :P
                        computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                        Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x ;
                        Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y +0.3;
                        printf("x:%f\n",Adjust_point.pose.position.x);
                        printf("y:%f\n",Adjust_point.pose.position.y);
                        Adjust_point.pose.position.z = 0.2;
                        Adjust_point.pose.orientation.x = 0;
                        Adjust_point.pose.orientation.y = 0;
                        Adjust_point.pose.orientation.z = 0;
                        Adjust_point.pose.orientation.w = 1;
                        ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                        // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x + 0.2;
                        // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y ;
                        // //yaw角设置
                        // Adjust_point.pose.orientation.x = 0;
                        // Adjust_point.pose.orientation.y = 0;
                        // Adjust_point.pose.orientation.z = 0;
                        // Adjust_point.pose.orientation.w = 1;
                        pos_pub.publish(Adjust_point);
                        second_adjust = false;
                        droping_second = false;
                        // ROS_INFO("3");
                    }
                }
                if(std::abs(pose_data.pose_local.pose.position.z - 0.2) < 0.05 && std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x) <0.03 && std::abs(pose_data.pose_local.pose.position.y - Adjust_point.pose.position.y)<0.03)
                { 
                     Ser_pub('P');
                    Drop_queue.pop();
                    // current_class.data = "";
                    ROS_INFO("Droping Finsh ");
                    droping_flag =  true;
                    droping_second = true;
                    second_adjust = true;
                    ros::Duration(1.5).sleep();
                    ros::spinOnce();
                    current_state = DroneState::HIGHING;
                    hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                    hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                    hight_point.pose.position.z = 1.0;
                    hight_point.pose.orientation.x = 0;
                    hight_point.pose.orientation.y = 0;
                    hight_point.pose.orientation.z = 0;
                    hight_point.pose.orientation.w = 1;
                    pos_pub.publish(hight_point);
                    // startDataCollection();
                }                     
            }
            else
            {
                //ROS_INFO("debug");const
                if(droping_i>0)
                {
                    droping_i--;
                    printf("num: %d",droping_i);
                }
                if(droping_i <=0)
                {
                    current_state = DroneState::LAND;
                    ROS_INFO("FAILED");
                    droping_i = 200;
                }
            }
            break;

        case DroneState::DROPING:
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            ROS_INFO("DROP");
            // judge_start.data =1;
            // judge_start_pub.publish(judge_start);
            if(!image_staff.image_data.detected_class.empty()) //|| !class_staff.class_data.classify_class.empty())
            {
                if(droping_flag)
                {
                    // 停止数据采集
                    // 停止数据采集
                    stopDataCollection();
                    // 使用基于类别统计的中位数进行微调
                    double median_x, median_y;
                    std::string target_class;            
                    if(calculateClassBasedMedianAdjustment(median_x, median_y, target_class)) 
                    {
                        // 使用统计得出的主要类别的中位数位置进行微调
                        Adjust_point.pose.position.x = median_x;
                        Adjust_point.pose.position.y = median_y;
                        Adjust_point.pose.position.z = 1.0;
                        Adjust_point.pose.orientation.x = 0;
                        Adjust_point.pose.orientation.y = 0;
                        Adjust_point.pose.orientation.z = 0;
                        Adjust_point.pose.orientation.w = 1;
                
                        // 更新当前类别为主要检测到的类别
                        current_class.data = target_class;
                        if ( target_class == "car")
                        {
                            dropped_classes.erase("car");
                        }
                        // else if (target_class == "random")
                        // {
                        //     dropped_classes.erase("random");
                        // }
                        else if (target_class == "bridge")
                        {
                            dropped_classes.erase("bridge");
                        }
                        else if (target_class == "tank" )
                        {
                            drop_random = true;
                            dropped_classes.erase("tank");

                        }
                        else if (target_class == "random" )
                        {
                            drop_random = true;
                            dropped_classes.erase("random");


                        }
                        
                        printf("Using filtered median for class '%s' - x:%.3f, y:%.3f\n remain class's count is %ld\n", 
                        target_class.c_str(), median_x, median_y,dropped_classes.size());
                       
                        ROS_INFO("Applied median adjustment based on current detection: %s", target_class.c_str()); image_staff.image_data.detected_class;                

                    } 
                    else 
                    {
                        // 如果基于类别的统计失败，使用当前检测进行微调
                        computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                                pose_data.pose_local.pose.position.z,
                                pose_data.pose_local.pose.orientation,
                                delta_x, delta_y, delta_z);
                                Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x;
                                Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y;
                                Adjust_point.pose.position.z = 1.0;
                                Adjust_point.pose.orientation.x = 0;
                                Adjust_point.pose.orientation.y = 0;
                                Adjust_point.pose.orientation.z = 0;
                                Adjust_point.pose.orientation.w = 1;
                            
                        current_class.data = image_staff.image_data.detected_class;
                        printf("Using current detection fallback - x:%.3f, y:%.3f\n", 
                                delta_x, delta_y);
                    }
                        
                    droping_flag = false;
                    printf("class:%s\n", current_class.data.c_str());
                    pos_pub.publish(Adjust_point);
                    droping_second = true;
                }
                // image_staff.image_data.detected_class=  class_staff.classfiy_data.data;
                if( droping_second && getLengthBetweenPoints(pose_data.pose_local.pose.position,Adjust_point.pose.position)<0.15)
                {
                    if ( goods_num == 0 || (image_staff.image_data.detected_class != "random" && image_staff.image_data.detected_class != "bridge" && image_staff.image_data.detected_class != "car" && !((mission_num == 2 && goods_num == 2) || (mission_num == 3 && goods_num == 1))))                    
                    {
                        current_state = DroneState::TRACKING_WAYPOINT;
                        mission_num+=1;  
                        printf("null or not ");
                        droping_flag = true;
                    }
                    else
                    {
                        printf("success\n");
                        ros::Duration(1.0).sleep();
                        ros::spinOnce();
                        if(Drop_queue.front() == 'C' && second_adjust)
                        {
                            //根据第一个进行调整 :C
                            computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                            Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x+0.2;
                            Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
                            printf("x:%f\n",Adjust_point.pose.position.x);
                            printf("y:%f\n",Adjust_point.pose.position.y);

                            Adjust_point.pose.position.z = 0.2;
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x + 0.2;
                            // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y ;
                            // //yaw角设置
                            // Adjust_point.pose.orientation.x = 0;
                            // Adjust_point.pose.orientation.y = 0;
                            // Adjust_point.pose.orientation.z = 0;
                            // Adjust_point.pose.orientation.w = 1;
                            pos_pub.publish(Adjust_point);
                            second_adjust = false;
                            droping_second = false;
                            ROS_INFO("3");
                        }
                        else if(Drop_queue.front() == 'U' && second_adjust)
                        {
                            //根据第二个进行调整 :U
                            ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                            Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x-0.2; //-0.2;
                            Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y;
                            Adjust_point.pose.position.z = 0.2;
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x -0.2;
                            // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y  ;
                            // // Adjust_point.pose.position.z =  0.5;
                            // // //yaw角设置
                            // Adjust_point.pose.orientation.x = 0;
                            // Adjust_point.pose.orientation.y = 0;
                            // Adjust_point.pose.orientation.z = 0;
                            // Adjust_point.pose.orientation.w = 1;
                            printf("x:%f\n",Adjust_point.pose.position.x);
                            printf("y:%f\n",Adjust_point.pose.position.y);
                            pos_pub.publish(Adjust_point);
                            second_adjust = false;
                            droping_second = false;
                            ROS_INFO("2");
                        }
                        // else if(Drop_queue.front() == 'U' && second_adjust)
                        // {
                        //     //根据第二个进行调整 :U
                        //     ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                        //     computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                        //     Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x-0.2 ;
                        //     Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
                        //     Adjust_point.pose.position.z = 0.2;
                        //     Adjust_point.pose.orientation.x = 0;
                        //     Adjust_point.pose.orientation.y = 0;
                        //     Adjust_point.pose.orientation.z = 0;
                        //     Adjust_point.pose.orientation.w = 1;
                        //     // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x -0.2;
                        //     // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y  ;
                        //     // // Adjust_point.pose.position.z =  0.5;
                        //     // // //yaw角设置
                        //     // Adjust_point.pose.orientation.x = 0;
                        //     // Adjust_point.pose.orientation.y = 0;
                        //     // Adjust_point.pose.orientation.z = 0;
                        //     // Adjust_point.pose.orientation.w = 1;
                        //     printf("x:%f\n",Adjust_point.pose.position.x);
                        //     printf("y:%f\n",Adjust_point.pose.position.y);
                        //     pos_pub.publish(Adjust_point);
                        //     second_adjust = false;
                        //     droping_second = false;
                        //     ROS_INFO("1");
                        // }
                    }
                }
                if(std::abs(pose_data.pose_local.pose.position.z - 0.2) < 0.05 && std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x) <0.03 && std::abs(pose_data.pose_local.pose.position.y - Adjust_point.pose.position.y)<0.03)
                { 
                    ROS_INFO_THROTTLE(1.0, "Data: %s", current_class.data.c_str());
                    if(mission_num ==3)
                    {
                        Ser_pub(Drop_queue.front());
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();
                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        startDataCollection();
                    }
                    else if(mission_num == 2)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        // goods_num--;   
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();
                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        goods_num--;
                        startDataCollection();
                    }
                    // if(mission_num == 1)
                    // {
                    //     Ser_pub(Drop_queue.front());
                    //     Drop_queue.pop();
                    //     ROS_INFO("Droping Finsh ");
                    //     droping_flag =  true;
                    //     // goods_num--;   
                    //     droping_second = true;
                    //     second_adjust = true;
                    //     ros::Duration(1.5).sleep();
                    //     ros::spinOnce();
                    //     current_state = DroneState::TRACKING_WAYPOINT;
                    //     hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                    //     hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                    //     hight_point.pose.position.z = 1.0;
                    //     hight_point.pose.orientation.x = 0;
                    //     hight_point.pose.orientation.y = 0;
                    //     hight_point.pose.orientation.z = 0;
                    //     hight_point.pose.orientation.w = 1;
                    //     pos_pub.publish(hight_point);
                    //     mission_num+=1;
                    //     goods_num--;
                    //     startDataCollection();
                    // }
                    else if(current_class.data =="bridge" )
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        ROS_INFO("Droping Finsh ");
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        goods_num--;
                        printf("num:%d\n",goods_num);
                        startDataCollection();
                        // current_class.data = "";
                    }
                    else if(current_class.data == "car" )
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        // current_class.data = "";
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();
                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        goods_num--;
                        printf("num:%d\n",goods_num);
                        startDataCollection();
                    }     
                    else if(current_class.data == "random" )
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        // current_class.data = "";
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();

                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        goods_num--;
                        printf("num:%d\n",goods_num);
                        startDataCollection();
                    } 
                    else if(current_class.data == "tank" )
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        // current_class.data = "";
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.5).sleep();
                        ros::spinOnce();

                        current_state = DroneState::TRACKING_WAYPOINT;
                        hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        hight_point.pose.position.z = 1.0;
                        hight_point.pose.orientation.x = 0;
                        hight_point.pose.orientation.y = 0;
                        hight_point.pose.orientation.z = 0;
                        hight_point.pose.orientation.w = 1;
                        pos_pub.publish(hight_point);
                        mission_num+=1;
                        goods_num--;
                        printf("num:%d\n",goods_num);
                        startDataCollection();
                    }               
                }                     
            }
            else
            {
                //ROS_INFO("debug");const
                if(droping_i>0)
                {
                    droping_i--;
                    printf("num: %d",droping_i);
                }
                if(droping_i <=0)
                {
                    current_state = DroneState::LAND;
                    ROS_INFO("FAILED");
                    droping_i = 200;
                }
            }
            break;
                     
        case DroneState::FINISH_Dynamic:
            //测试点
            finish_Point.pose.position =drop_finish_point.pose.position;
            //正式点
            // finish_Point.pose.position.x =7.4;
            // finish_Point.pose.position.y = 2.3;
            // finish_Point.pose.position.z = 1.0;
            finish_Point.pose.orientation.x = 0;
            finish_Point.pose.orientation.y = 0;
            finish_Point.pose.orientation.z = 0;
            finish_Point.pose.orientation.w = 1;
            if (ego_contral)
            {
                position_pub.publish(finish_Point);
                ego_contral = false;
            }
            
            pos_pub.publish(finish_Point);
            ROS_INFO("FINISH DYNAMIC");
            if (std::abs(pose_data.pose_local.pose.position.x - finish_Point.pose.position.x) < 0.1 &&
                std::abs(pose_data.pose_local.pose.position.y - finish_Point.pose.position.y) < 0.1 &&
                std::abs(pose_data.pose_local.pose.position.z - finish_Point.pose.position.z) < 0.1)
            {
                // ros::Duration(1.0).sleep();
                //测试状态
                // current_state = DroneState::LAND;
                //正式状态
                current_state = DroneState::CROSS_LAND;
                ego_contral = true;
            }           
            break; 

        case DroneState::CROSS_LAND:
            // cross_land_point.pose.position.x = 7.4;
            // cross_land_point.pose.position.y = 2.3;
            // cross_land_point.pose.position.z = 0.6;

            cross_land_point.pose.position.x= finish_Point.pose.position.x;
            cross_land_point.pose.position.y= finish_Point.pose.position.y;
            cross_land_point.pose.position.z= 0.6;
            cross_land_point.pose.orientation.x = 0;
            cross_land_point.pose.orientation.y = 0;
            cross_land_point.pose.orientation.z = 0;
            cross_land_point.pose.orientation.w = 1; 
            pos_pub.publish(cross_land_point);
            ROS_INFO("----land----");
            if (std::abs(pose_data.pose_local.pose.position.z - 0.6) < 0.05)
            {
                current_state = DroneState::DECIDE_CROSS;
            }
            break;
        case DroneState::DECIDE_CROSS:
            // cross_point.pose.orientation = change_yaw_point.pose.orientation;
            if (cross_judge)
            {
                cross_pub.publish(cross_point);
                ros::Duration(1.0).sleep();
                ros::spinOnce();

            }
            pos_pub.publish(cross_point);
            current_state = DroneState::JUDGE_CROSS;
            // current_state = DroneState::LAND;
            ROS_INFO("-send-");
            // cross_judge = false;
            break;

        case DroneState::JUDGE_CROSS:
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_point.pose.position) < 0.25)
            {
                ROS_INFO("---success---");
                current_state = DroneState::DECIDE_CROSS02;
                ros::Duration(3.0).sleep();
                ros::spinOnce();
                // current_state = DroneState::LAND;

            }
            ROS_INFO("---gonging---");
            break;
        case DroneState::DECIDE_CROSS02:
            if (cross_judge)
            {
                cross_pub.publish(cross_point_02);
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
            pos_pub.publish(cross_point_02);
            current_state = DroneState::JUDGE_CROSS02;
            ROS_INFO("-send-");
            cross_judge = false;
            break;

        
        case DroneState::JUDGE_CROSS02:
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_point_02.pose.position) < 0.2)
            {
                ROS_INFO("---success---");
                current_state = DroneState::LAND;

            }
            ROS_INFO("---gonging---");
            break;

        case DroneState::LAND:
                //正式点n.x = cross_point.pose.position.x;
                // land_point.pose.position.y = cross_point.pose.position.y;
                // land_point.pose.position.z = -0.01;
                if(true)
                {
                    //测试点
                    land_point.pose.position.x = pose_data.pose_local.pose.position.x;
                    land_point.pose.position.y = pose_data.pose_local.pose.position.y;
                    land_point.pose.position.z = 0;
                    land_point.pose.orientation = cross_point_02.pose.orientation;
                    // land_point.pose.orientation.x = 0;
                    // land_point.pose.orientation.y = 0;
                    // land_point.pose.orientation.z =0;
                    // land_point.pose.orientation.w = 1;
                    current_state = DroneState::FINISH;
                    pos_pub.publish(land_point);
                    ROS_INFO("--LANDING---");
                }
            break;

        case DroneState::FINISH:
            if(std::abs(pose_data.pose_local.pose.position.z - 0)<0.05)
            {
                ros::Duration(1.0);
                enableEmergency();
                ROS_INFO("Mission Complete!");
                ros::shutdown();
            }
            break;
    }
}
void MissionFSM::GetParameters(const ros::NodeHandle& nh)
{
    parameters_.load(nh);
}

bool MissionFSM::isKnownTargetClass(const std::string& class_name) const
{
    return class_name == "random" || class_name == "tank" ||
           class_name == "car" || class_name == "bridge" ||
           class_name == "tent" || class_name == "bunker";
}

bool MissionFSM::isHighPriorityClass(const std::string& class_name) const
{
    return class_name == "random" || class_name == "tank" || class_name == "car";
}

void MissionFSM::recordSearchDetection()
{
    const std::string class_name = image_staff.image_data.detected_class;
    if (!isKnownTargetClass(class_name) ||
        remaining_classes_.count(class_name) == 0 ||
        cached_targets_.count(class_name) != 0 ||
        image_staff.image_data.cx == 0 || image_staff.image_data.cy == 0) {
        return;
    }

    double correction_x = 0.0;
    double correction_y = 0.0;
    double correction_z = 0.0;
    computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy,
                      pose_data.pose_local.pose.position.z,
                      pose_data.pose_local.pose.orientation,
                      correction_x, correction_y, correction_z);

    std::vector<std::pair<double, double>>& samples = target_samples_[class_name];
    samples.push_back(std::make_pair(
        pose_data.pose_local.pose.position.x + correction_x,
        pose_data.pose_local.pose.position.y + correction_y));
    if (samples.size() > RANDOM_SAMPLE_THRESHOLD) {
        samples.erase(samples.begin());
    }

    if (samples.size() >= RANDOM_SAMPLE_THRESHOLD && buildStableTarget(class_name) &&
        isHighPriorityClass(class_name) && active_search_class_.empty()) {
        beginSearchTarget(class_name, cached_targets_[class_name]);
    }
}

bool MissionFSM::buildStableTarget(const std::string& class_name)
{
    std::map<std::string, std::vector<std::pair<double, double>>>::iterator found =
        target_samples_.find(class_name);
    if (found == target_samples_.end() ||
        found->second.size() < RANDOM_SAMPLE_THRESHOLD) {
        return false;
    }

    std::vector<double> x_values;
    std::vector<double> y_values;
    for (std::size_t i = 0; i < found->second.size(); ++i) {
        x_values.push_back(found->second[i].first);
        y_values.push_back(found->second[i].second);
    }
    std::sort(x_values.begin(), x_values.end());
    std::sort(y_values.begin(), y_values.end());
    const std::size_t middle = x_values.size() / 2;
    const double median_x = x_values.size() % 2 == 0
        ? (x_values[middle - 1] + x_values[middle]) * 0.5 : x_values[middle];
    const double median_y = y_values.size() % 2 == 0
        ? (y_values[middle - 1] + y_values[middle]) * 0.5 : y_values[middle];

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "camera_init";
    target.pose.position.x = median_x;
    target.pose.position.y = median_y;
    target.pose.position.z = parameters_.search_height;
    target.pose.orientation.w = 1.0;
    cached_targets_[class_name] = target;
    found->second.clear();
    ROS_INFO("Stable search target '%s' cached at (%.3f, %.3f)",
             class_name.c_str(), median_x, median_y);
    return true;
}

bool MissionFSM::selectCachedLowPriorityTarget()
{
    static const char* priority[] = {"bridge", "tent", "bunker"};
    for (std::size_t i = 0; i < sizeof(priority) / sizeof(priority[0]); ++i) {
        const std::string class_name(priority[i]);
        std::map<std::string, geometry_msgs::PoseStamped>::const_iterator target =
            cached_targets_.find(class_name);
        if (remaining_classes_.count(class_name) != 0 && target != cached_targets_.end()) {
            beginSearchTarget(class_name, target->second);
            return true;
        }
    }
    return false;
}

void MissionFSM::beginSearchTarget(const std::string& class_name,
                                   const geometry_msgs::PoseStamped& target)
{
    active_search_class_ = class_name;
    active_search_target_ = target;
    search_adjust_phase_ = 0;
    search_goal_sent_ = false;
    search_phase_started_ = ros::Time::now();
    current_state = DroneState::APPROACH_DETECTED_TARGET;
    ROS_INFO("Search interrupted for target '%s'", class_name.c_str());
}

void MissionFSM::completeSearchDrop()
{
    if (Drop_queue.empty()) {
        ROS_ERROR("Drop queue is empty; finishing search without release");
        current_state = DroneState::SEARCH_FINISHED;
        return;
    }

    Ser_pub(Drop_queue.front());
    Drop_queue.pop();
    remaining_classes_.erase(active_search_class_);
    cached_targets_.erase(active_search_class_);
    target_samples_.erase(active_search_class_);
    ++completed_drops_;
    ROS_INFO("Dropped '%s' (%d/3)", active_search_class_.c_str(), completed_drops_);
    active_search_class_.clear();
    search_adjust_phase_ = 0;
    search_goal_sent_ = false;
    current_state = completed_drops_ >= 3
        ? DroneState::SEARCH_FINISHED : DroneState::RESUME_SEARCH;
}

void MissionFSM::cancelSearchTarget()
{
    ROS_WARN("Target '%s' lost during adjustment; returning to search",
             active_search_class_.c_str());
    target_samples_.erase(active_search_class_);
    cached_targets_.erase(active_search_class_);
    active_search_class_.clear();
    search_adjust_phase_ = 0;
    search_goal_sent_ = false;
    current_state = DroneState::RESUME_SEARCH;
}

void MissionFSM::finishSearchMission()
{
    finish_Point.header.frame_id = "camera_init";
    finish_Point.pose.position.x = pose_data.pose_local.pose.position.x;
    finish_Point.pose.position.y = pose_data.pose_local.pose.position.y;
    finish_Point.pose.position.z = parameters_.search_height;
    finish_Point.pose.orientation.x = 0.0;
    finish_Point.pose.orientation.y = 0.0;
    finish_Point.pose.orientation.z = 0.0;
    finish_Point.pose.orientation.w = 1.0;
    ego_contral = true;
    current_state = DroneState::FINISH_Dynamic;
}
void MissionFSM::pose_pub(const std::vector<geometry_msgs::PoseStamped>& target_points,int flag)
{
    static bool trj_judge = true;
    static ros::Time last_request = ros::Time::now();  // 确保初始化
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_request = current_time - last_request;
    static geometry_msgs::PoseStamped active_target;  // **新增：保存当前激活的目标点**
    // 检查 target_points 是否包含足够的点
    if (target_points.size() < 5) {
        ROS_ERROR("target_points size is less than 5. Current size: %zu", target_points.size());
    }
    if (true)
    {       
            last_request = current_time;
            if(trj_judge)
            {
                 if (use_random_median && mission_num !=4 && mission_num !=3) 
                 {
                    active_target = random_median_target;  // **保存random目标**
                    position_pub.publish(random_median_target);
                    pos_pub.publish(random_median_target);
                    ROS_INFO("Using random median target instead of waypoint #%d", flag);
                    
                    // 重置标志,以便下次可以重新收集
                    use_random_median = false;
                    random_positions_1.clear();
                    random_positions_2.clear();  // 新增:初始化第二个点集

                } 
                else 
                {
                    active_target = target_points[flag];  // **保存普通航点**
                    position_pub.publish(target_points[flag]);
                    pos_pub.publish(target_points[flag]);
                }


                // position_pub.publish(target_points[flag]);
                // pos_pub.publish(target_points[flag]);
                // class_staff.classfiy_data.data = "";
                // class_staff.confidence_ = 0;
                trj_judge = false;
                //开启航点的采集
                startDataCollection();
            }
     // **修改: 根据是否使用random目标选择比较点**
       // 使用保存的active_target进行位置判断
            pos_pub.publish(active_target);

            if(std::abs(pose_data.pose_local.pose.position.x - active_target.pose.position.x) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.y - active_target.pose.position.y) < 0.1 && 
            std::abs(pose_data.pose_local.pose.position.z - active_target.pose.position.z) < 0.1) 
            {
                current_state = DroneState::DROPING;
                trj_judge = true;
            }
    }
}


//识别二维码的第一次目标点
bool MissionFSM::first_pub(const std::vector<geometry_msgs::PoseStamped>& points) 
{
    static int flag = 1;
    static ros::Time last_request = ros::Time::now();  // 确保初始化
    bool judge = false;
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_request = current_time - last_request;
    if (true) 
    {
        switch (flag) {
            case 1:
                flag = 2;
                position_pub.publish(points[0]);
                pos_pub.publish(points[0]);
                break;
            case 2:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.05 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.05 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.05)
                {  // 使用 flag - 1
                    flag = 3;
                    position_pub.publish(points[1]);
                    pos_pub.publish(points[1]);
                    ROS_INFO("------FIRST-----");
                }
                break;
            case 3:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.05 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.05 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.05)
                {  // 使用 flag - 1
                    flag = 4;
                    position_pub.publish(points[2]);
                    pos_pub.publish(points[2]);
                    // judge = true;
                    // ROS_INFO("FINSH");
                }
                break;
            case 4:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.1 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.1 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.1)
                {  // 使用 flag - 1
                    flag = 5;
                    position_pub.publish(points[3]);
                    pos_pub.publish(points[3]);
                }
                break;
                case 5:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.1 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.1 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.1)
                {  // 使用 flag - 1
                    // flag = 5;
                    // position_pub.publish(points[3]);
                    // pos_pub.publish(points[3]);
                    judge=true;
                    current_state=DroneState::LAND;
                }
                break;
            default:
                ROS_ERROR("Unexpected flag value: %d", flag);
                break;
        }
    }
    return judge;
}

void MissionFSM::Ser_pub(uint8_t num) {
    // 静态变量只初始化一次，应该放在函数外部作为类成员
    static serial::Serial ser;
    static bool initialized = false;
    
    // 初始化串口（只执行一次）
    if(!initialized) {
        try {
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            
            if(ser.isOpen()) {
                ROS_INFO_STREAM("Serial Port initialized successfully");
                initialized = true;
            } 
            else {
                ROS_ERROR_STREAM("Failed to open serial port");
                return;
            }
        } catch (const serial::IOException& e) {
            ROS_ERROR_STREAM("Port open error: " << e.what());
            return;
        }
    }
    // 检查串口状态
    if(!ser.isOpen()) {
        ROS_WARN_STREAM("Serial port not available");
        return;
    }
    // 发送数据
    try {
            ser.write(&num, 1);
            ROS_DEBUG_STREAM("Sent big_drop");

    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Write failed: " << e.what());
    }
}

void MissionFSM::computeAdjustment(double u, double v, double depth, const geometry_msgs::Quaternion& ros_quat, 
    double& delta_x, double& delta_y, double& delta_z) 
{
    //四元数的准备工作
    if(u==0 && v==0)
    {
        delta_x = 0;
        delta_y = 0;
        delta_z = 0;
        return;
    }
     tf2::Quaternion tf_quat;
     tf_quat.setX(ros_quat.x);
     tf_quat.setY(ros_quat.y);
     tf_quat.setZ(ros_quat.z);
     tf_quat.setW(ros_quat.w);
     double roll, pitch, yaw;
     tf2::Matrix3x3 mat(tf_quat);
     mat.getRPY(roll, pitch, yaw);  // yaw 是弧度值
    // (1) 图像像素坐标 → 相机坐标系
    double x_cam =  ((u - parameters_.cx) / parameters_.fx) * (depth+0.175);
    double y_cam =  -((v - parameters_.cy) / parameters_.fy) * (depth+0.175);
    double z_cam =  -depth;
    //图像到无人机坐标
    delta_x = cos(yaw)*(y_cam + parameters_.dx) - (-x_cam + parameters_.dy)*sin(yaw);
    delta_y = cos(yaw)*(-x_cam + parameters_.dy) + (y_cam + parameters_.dx)*sin(yaw);
    // delta_x = cos(yaw)*(y_cam ) - (-x_cam  )*sin(yaw);
    // delta_y = cos(yaw)*(-x_cam ) + (y_cam )*sin(yaw);

}
 void MissionFSM::enableEmergency()
 {
    mavros_msgs::CommandLong emergency_srv;
    emergency_srv.request.command = 400;
    emergency_srv.request.param2 = 21196;
    arming_client.call(emergency_srv);
    ROS_INFO("FININSH LAND");
 }
 double MissionFSM::quaternionToYaw(const geometry_msgs::Quaternion& quat) {
    // 提取四元数各分量
    const double qx = quat.x;
    const double qy = quat.y;
    const double qz = quat.z;
    const double qw = quat.w;

    // 计算yaw角的分子和分母部分
    const double siny_cosp = 2 * (qw * qz + qx * qy);
    const double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);  // 简化后的表达式

    // 使用atan2计算yaw角（单位：弧度）
    return std::atan2(siny_cosp, cosy_cosp);
}


double MissionFSM::getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
    double *out_err_x , double *out_err_y , double *out_err_z )
{
    double err_x = a.x - b.x;
    double err_y = a.y - b.y;
    double err_z = a.z - b.z;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
}
// 在类的末尾修改这些函数（大约在第1050行之后）

// 添加新的delta对到向量
//yaw角判断
double MissionFSM::getYawDifference(const geometry_msgs::Quaternion& quat1, 
                                   const geometry_msgs::Quaternion& quat2) 
{
    // 分别计算两个四元数的yaw角
    double yaw1 = quaternionToYaw(quat1);
    double yaw2 = quaternionToYaw(quat2);
    
    // 计算角度差值
    double diff = yaw2 - yaw1;
    
    // 将角度差值规范化到 [-π, π] 范围内
    while (diff > M_PI) {
        diff -= 2.0 * M_PI;
    }
    while (diff < -M_PI) {
        diff += 2.0 * M_PI;
    }
    
    return diff;
}

// 如果你需要角度差值的绝对值（总是正数）
double MissionFSM::getAbsYawDifference(const geometry_msgs::Quaternion& quat1, 
                                      const geometry_msgs::Quaternion& quat2) 
{
    return std::abs(getYawDifference(quat1, quat2));
}

// 如果你需要返回度数而不是弧度
double MissionFSM::getYawDifferenceDegrees(const geometry_msgs::Quaternion& quat1, 
                                          const geometry_msgs::Quaternion& quat2) 
{
    double diff_rad = getYawDifference(quat1, quat2);
    return diff_rad * 180.0 / M_PI;  // 弧度转度数
}

//


// 新增：开始数据采集
void MissionFSM::startDataCollection() {
    is_collecting_data = true;
    flight_data_samples.clear();
    data_collection_start_time = ros::Time::now();
    ROS_INFO("Started collecting flight data");
}

// 新增：开启数据采集
void MissionFSM::stopDataCollection() {
    is_collecting_data = false;
    ROS_INFO("Stopped collecting flight data. Total samples: %zu", flight_data_samples.size());
}
void MissionFSM::collectFlightData() {
    if (!is_collecting_data) return;
    
    if (image_staff.image_data.detected_class != "null" && 
        (image_staff.image_data.cx != 0 && image_staff.image_data.cy != 0) && 
        (!image_staff.image_data.detected_class.empty()) && 
        checkWithCount(image_staff.image_data.detected_class))    
    {
        double temp_dx, temp_dy, temp_dz;
        computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy, 
                         pose_data.pose_local.pose.position.z, 
                         pose_data.pose_local.pose.orientation, 
                         temp_dx, temp_dy, temp_dz);
        
        FlightDataSample sample;
        sample.tar_x = temp_dx + pose_data.pose_local.pose.position.x;
        sample.tar_y = temp_dy + pose_data.pose_local.pose.position.y;
        sample.cur_class.data = image_staff.image_data.detected_class;
        sample.timestamp = ros::Time::now();
        sample.drone_position = pose_data.pose_local.pose.position;
        
        flight_data_samples.push_back(sample);

        // 根据检测到的类别分别存储到不同的点集
        if (image_staff.image_data.detected_class == "random") 
        {
            random_positions_1.push_back(std::make_pair(sample.tar_x, sample.tar_y));
            ROS_INFO("Collected RANDOM position #%zu: (%.3f, %.3f)", 
                     random_positions_1.size(), sample.tar_x, sample.tar_y);
            
            // 当任一点集达到阈值时,尝试计算中位数
            if ((random_positions_1.size() >= RANDOM_SAMPLE_THRESHOLD || 
                 random_positions_2.size() >= RANDOM_SAMPLE_THRESHOLD) && 
                !use_random_median) 
            {
                calculateRandomMedianTarget();
            }
        }
        else if (image_staff.image_data.detected_class == "tank") 
        {
            random_positions_2.push_back(std::make_pair(sample.tar_x, sample.tar_y));
            ROS_INFO("Collected TANK position #%zu: (%.3f, %.3f)", 
                     random_positions_2.size(), sample.tar_x, sample.tar_y);
            
            // 当任一点集达到阈值时,尝试计算中位数
            if ((random_positions_1.size() >= RANDOM_SAMPLE_THRESHOLD || 
                 random_positions_2.size() >= RANDOM_SAMPLE_THRESHOLD) && 
                !use_random_median) 
            {
                calculateRandomMedianTarget();
            }
        }
        
        if (flight_data_samples.size() > 1000) {
            flight_data_samples.erase(flight_data_samples.begin());
        }
    }
}

bool MissionFSM::calculateClassBasedMedianAdjustment(double& median_dx, double& median_dy, std::string& target_class) {
    if (flight_data_samples.empty()) {
        ROS_WARN("No flight data samples available for median calculation");
        return false;
    }
    
    // 获取当前检测到的类别
    std::string current_detected_class = image_staff.image_data.detected_class;
    
    if (current_detected_class.empty()) {
        ROS_WARN("No current class detected for filtering");
        return false;
    }
    
    // 验证当前检测的类别是否为有效目标
    if (current_detected_class != "bunker" && current_detected_class != "car" &&
        current_detected_class != "bridge" && current_detected_class != "tent" &&
        current_detected_class != "random" && current_detected_class != "tank") {
        ROS_WARN("Current detected class '%s' is not a valid target", current_detected_class.c_str());
        return false;
    }
    
    ROS_INFO("Filtering flight data for current detected class: %s", current_detected_class.c_str());
    
    // 统计所有类别的数据（用于调试信息）
    std::map<std::string, int> class_counts;
    std::vector<std::pair<double, double>> target_class_positions;
    
    // 筛选出与当前检测类别匹配的样本
    for (const auto& sample : flight_data_samples) {
        if (!sample.cur_class.data.empty()) {
            class_counts[sample.cur_class.data]++;
            
            // 只收集与当前检测类别匹配的位置数据
            if (sample.cur_class.data == current_detected_class) {
                target_class_positions.push_back(std::make_pair(sample.tar_x, sample.tar_y));
            }
        }
    }
    
    // 打印统计信息
    ROS_INFO("Flight data statistics:");
    for (const auto& pair : class_counts) {
        ROS_INFO("  %s: %d detections", pair.first.c_str(), pair.second);
    }
    
    // 检查是否有足够的目标类别数据
    if (target_class_positions.empty()) {
        ROS_WARN("No samples found for current detected class '%s' in flight data", current_detected_class.c_str());
        return false;
    }
    
    if (target_class_positions.size() < 3) {
        ROS_WARN("Only %zu samples found for class '%s', may not be reliable for median calculation", 
                 target_class_positions.size(), current_detected_class.c_str());
        // 可以选择继续计算或返回false，这里选择继续
    }
    
    // 提取X和Y坐标
    std::vector<double> tar_x_values, tar_y_values;
    for (const auto& position : target_class_positions) {
        tar_x_values.push_back(position.first);
        tar_y_values.push_back(position.second);
    }
    
    // 计算中位数的lambda函数
    auto calculateMedian = [](std::vector<double>& values) -> double {
        if (values.empty()) return 0.0;
        
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        
        if (n % 2 == 0) {
            return (values[n/2 - 1] + values[n/2]) / 2.0;
        } else {
            return values[n/2];
        }
    };
    
    median_dx = calculateMedian(tar_x_values);
    median_dy = calculateMedian(tar_y_values);
    target_class = current_detected_class;
    
    ROS_INFO("Calculated median position for current detected class '%s' from %zu samples: x=%.3f, y=%.3f", 
             target_class.c_str(), target_class_positions.size(), median_dx, median_dy);
    
    // 可选：打印位置数据的分布情况（用于调试）
    if (target_class_positions.size() > 1) {
        // 计算标准差以评估数据的离散程度
        double mean_x = 0, mean_y = 0;
        for (const auto& pos : target_class_positions) {
            mean_x += pos.first;
            mean_y += pos.second;
        }
        mean_x /= target_class_positions.size();
        mean_y /= target_class_positions.size();
        
        double var_x = 0, var_y = 0;
        for (const auto& pos : target_class_positions) {
            var_x += (pos.first - mean_x) * (pos.first - mean_x);
            var_y += (pos.second - mean_y) * (pos.second - mean_y);
        }
        var_x /= target_class_positions.size();
        var_y /= target_class_positions.size();
        
        double std_x = sqrt(var_x);
        double std_y = sqrt(var_y);
        
        ROS_INFO("Position data distribution - Mean: (%.3f, %.3f), Std: (%.3f, %.3f)", 
                 mean_x, mean_y, std_x, std_y);
        
        // 如果标准差太大，发出警告
        if (std_x > 0.5 || std_y > 0.5) {
            ROS_WARN("High position variance detected - median may not be reliable");
        }
    }
    
    return true;
}

bool MissionFSM::checkWithCount(const std::string& class_name) {
    // count() 对于 set 来说只能返回 0 或 1 (因为set不允许重复)
    // 0 = 不存在, 1 = 存在
    return dropped_classes.count(class_name) > 0;
    
    // 或者更简洁地写成:
    // return dropped_classes.count(class_name);  // 0为false, 1为true
}
// 修改点3: calculateRandomMedianTarget函数 - 比较两个点集并选择数量多的
void MissionFSM::calculateRandomMedianTarget() {
    // 比较两个点集的大小,选择样本数量更多的
    std::vector<std::pair<double, double>>* target_positions = nullptr;
    std::string selected_class;
    
    size_t random_count = random_positions_1.size();
    size_t tank_count = random_positions_2.size();
    
    ROS_INFO("Comparing samples - Random: %zu, Tank: %zu", random_count, tank_count);
    
    // 选择数量更多的点集
    if (random_count >= tank_count && random_count >= RANDOM_SAMPLE_THRESHOLD) {
        target_positions = &random_positions_1;
        selected_class = "random";
        ROS_INFO("Selected RANDOM class (more samples)");
    } 
    else if (tank_count > random_count && tank_count >= RANDOM_SAMPLE_THRESHOLD) {
        target_positions = &random_positions_2;
        selected_class = "tank";
        ROS_INFO("Selected TANK class (more samples)");
    }
    else {
        // 两个点集都不满足阈值要求
        ROS_WARN("Neither class has enough samples. Random: %zu/%zu, Tank: %zu/%zu", 
                 random_count, RANDOM_SAMPLE_THRESHOLD,
                 tank_count, RANDOM_SAMPLE_THRESHOLD);
        return;
    }
    
    // 提取X和Y坐标
    std::vector<double> x_values, y_values;
    for (const auto& pos : *target_positions) {
        x_values.push_back(pos.first);
        y_values.push_back(pos.second);
    }
    
    // 计算中位数的lambda函数
    auto calculateMedian = [](std::vector<double>& values) -> double {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        if (n % 2 == 0) {
            return (values[n/2 - 1] + values[n/2]) / 2.0;
        } else {
            return values[n/2];
        }
    };
    
    double median_x = calculateMedian(x_values);
    double median_y = calculateMedian(y_values);
    
    // 设置random中位数目标点
    random_median_target.pose.position.x = median_x;
    random_median_target.pose.position.y = median_y;
    random_median_target.pose.position.z = 1.2;
    random_median_target.pose.orientation.x = 0;
    random_median_target.pose.orientation.y = 0;
    random_median_target.pose.orientation.z = 0;
    random_median_target.pose.orientation.w = 1;
    
    // 根据选中的类别设置标志并从dropped_classes中移除
    if (checkWithCount(selected_class)) {
        use_random_median = true;
        dropped_classes.erase(selected_class);
        ROS_INFO("Set use_random_median=true for class: %s", selected_class.c_str());
    }
    
    ROS_INFO("Calculated %s median target from %zu samples: (%.3f, %.3f)", 
             selected_class.c_str(), target_positions->size(), median_x, median_y);
    
    // 打印统计信息
    double mean_x = 0, mean_y = 0;
    for (const auto& pos : *target_positions) {
        mean_x += pos.first;
        mean_y += pos.second;
    }
    mean_x /= target_positions->size();
    mean_y /= target_positions->size();
    
    // 计算标准差
    double var_x = 0, var_y = 0;
    for (const auto& pos : *target_positions) {
        var_x += (pos.first - mean_x) * (pos.first - mean_x);
        var_y += (pos.second - mean_y) * (pos.second - mean_y);
    }
    var_x /= target_positions->size();
    var_y /= target_positions->size();
    double std_x = sqrt(var_x);
    double std_y = sqrt(var_y);
    
    ROS_INFO("%s position statistics:", selected_class.c_str());
    ROS_INFO("  Mean: (%.3f, %.3f)", mean_x, mean_y);
    ROS_INFO("  Median: (%.3f, %.3f)", median_x, median_y);
    ROS_INFO("  Std Dev: (%.3f, %.3f)", std_x, std_y);
    
    if (std_x > 0.5 || std_y > 0.5) {
        ROS_WARN("High position variance detected for %s - median may not be reliable", 
                 selected_class.c_str());
    }
}

double MissionFSM::Point_ToYAW(geometry_msgs::PoseStamped point_1,geometry_msgs::PoseStamped point_2)
{
    double yaw = atan2(point_2.pose.position.y - point_1.pose.position.y,
                       point_2.pose.position.x - point_1.pose.position.x);
    return yaw;

}
void MissionFSM::Point_ToYAW_WithQuaternion(geometry_msgs::PoseStamped point_1, 
                                   geometry_msgs::PoseStamped point_2,
                                   geometry_msgs::Quaternion& quaternion)
{
    // 计算yaw角
    double yaw = atan2(point_2.pose.position.y - point_1.pose.position.y,
                       point_2.pose.position.x - point_1.pose.position.x);
    
    // 将yaw角转换为四元数（绕z轴旋转）
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);  // roll=0, pitch=0, yaw=计算得到的角度
    
    // 转换为geometry_msgs::Quaternion
    quaternion.x = q.x();
    quaternion.y = q.y();
    quaternion.z = q.z();
    quaternion.w = q.w();
    
    // return yaw;
}
double MissionFSM::calculateMovingDirection(const std::vector<geometry_msgs::Point>& positions) {
    if (positions.size() < 2) return 0.0;
    
    // 计算平均值
    double mean_x = 0, mean_y = 0;
    for (const auto& p : positions) {
        mean_x += p.x;
        mean_y += p.y;
    }
    mean_x /= positions.size();
    mean_y /= positions.size();
    
    // 线性回归: y = kx + b, 求斜率k
    double numerator = 0, denominator = 0;
    for (const auto& p : positions) {
        numerator += (p.x - mean_x) * (p.y - mean_y);
        denominator += (p.x - mean_x) * (p.x - mean_x);
    }
    
    if (std::abs(denominator) < 1e-6) {
        // 垂直运动,沿Y轴
        // 返回与Y轴垂直的方向（X轴方向）
        return (positions.back().y > positions.front().y) ? 0.0 : M_PI;
    }
    
    double slope = numerator / denominator;
    double yaw = atan(slope);
    
    // 根据X方向调整角度
    if (positions.back().x < positions.front().x) {
        yaw += M_PI;
    }
    
    // 计算垂直方向的角度（原角度+90度）
    double perpendicular_yaw = yaw + M_PI/2;
    
    // 规范化角度到[-π, π]范围
    while (perpendicular_yaw > M_PI) {
        perpendicular_yaw -= 2 * M_PI;
    }
    while (perpendicular_yaw < -M_PI) {
        perpendicular_yaw += 2 * M_PI;
    }
    
    return perpendicular_yaw;
}
