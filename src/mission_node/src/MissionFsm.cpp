#include "input.h"
#include "MissionFsm.h"
// #include "input.h"
#include <quadrotor_msgs/TakeoffLand.h>
#include "cmath"
#include <vector>
#include <algorithm>
//U:y+0.2
const double fx = 1092.34009;  // 焦距（像素）
const double fy = 1088.58832;
const double cx = 657.880369;
const double cy = 361.681183;
const double dx = 0;    // 相机在无人机机体坐标系的偏移（x: 右，y: 前，z: 上）
const double dy = 0.21;
const double dz = 0.0;
const float max_velocity_x = 0.5;
const float max_velocity_y = 0.5;
const float drop_height=1.0;
const float g=9.8;
const float land_vel = 0.2;
// const float drop_time=sqrt(2*drop_height /g);
const float drop_time = drop_height/land_vel; // 投放时间
const size_t REQUIRED_DATA_COUNT = 20; // 需要20个数据对
const double HISTORY_DURATION = 5.0; // 5秒时间窗口

//调整量
double delta_x, delta_y, delta_z;
geometry_msgs::Quaternion debug_quat;

//微调位置
geometry_msgs::PoseStamped Adjust_point;

MissionFSM::MissionFSM() : rate(20.0) {
    // 其他初始化
    current_state = DroneState::INIT;
    // current_state = DroneState::DECIDE_CROSS;
    current_drone_state = DyDropState::TRACKING;
    Drop_queue.push('C');
    Drop_queue.push('P');
    Drop_queue.push('U');
    mission_num = 0;
    linear_x_p = 0.3;
    linear_y_p = 0.3;
    // linear_x_d = linear_y_d = 0.05;
    droping_flag = true ;
    droping_second = false;
    second_adjust = true;
    yaw_judge = false;
    cross_judge = true;
    cargo_dropped = false;
    ego_contral = true;
    droping_i = 300; 
    goods_num = 3;
    last_target_x=0;
    last_target_y=0;
    dropped_classes.insert("car");
    dropped_classes.insert("bridge");
    dropped_classes.insert("bunker");
    // dropped_classes.push_back("car");


    is_collecting_data = false;
    flight_data_samples.clear();

    cross_point.header.frame_id = "camera_init";
    // cross_point.pose.position.x = 9.0;
    // cross_point.pose.position.y = 1.95;
    // cross_point.pose.position.z = 0.6;
    cross_point.pose.position.x = -2.0;//9.0;
    cross_point.pose.position.y = 9.0;//2.0;
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

    cross_point_02.header.frame_id = "camera_init";
    // cross_point_02.pose.position.x = 9.0;
    // cross_point_02.pose.position.y = -0.65;
    // cross_point_02.pose.position.z = 0.6;
    cross_point_02.pose.position.x = 0.48;//9.1;
    cross_point_02.pose.position.y = 9.1;//-0.48;
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
    //投货的目标位置

    // 第1个目标点
    geometry_msgs::PoseStamped target_3;
    target_3.pose.position.x = 2.55;//1.76;
    target_3.pose.position.y = 1.76;//-2.55;
    target_3.pose.position.z = 1.0;
    //yaw角设置1
    target_3.pose.orientation.x = 0;
    target_3.pose.orientation.y = 0;
    target_3.pose.orientation.z = 0;
    target_3.pose.orientation.w = 1;

    target_points.push_back(target_3);

    // 第2个目标点
    geometry_msgs::PoseStamped target_4;
    target_4.pose.position.x = 1.7;//4.5;
    target_4.pose.position.y = 4.5;//-1.7;
    target_4.pose.position.z = 1.0;

    target_4.pose.orientation.x = 0;
    target_4.pose.orientation.y = 0;
    target_4.pose.orientation.z = 0;
    target_4.pose.orientation.w = 1;
    target_points.push_back(target_4);
       // 第3个目标点
    geometry_msgs::PoseStamped target_5;
    target_5.pose.position.x = -2.32;//2.46;

    target_5.pose.position.y = 2.46;//2.32;

    target_5.pose.position.z = 1.0;
    //yaw角设置
    target_5.pose.orientation.x = 0;
    target_5.pose.orientation.y = 0;
    target_5.pose.orientation.z = 0;
    target_5.pose.orientation.w = 1;
    target_points.push_back(target_5);

    // 第4个目标点
    geometry_msgs::PoseStamped target_1;
    target_1.pose.position.x = -1.3;//4.6;
    target_1.pose.position.y = 4.6;//1.3;
    target_1.pose.position.z = 1.0;

    //yaw角设置
    target_1.pose.orientation.x = 0;
    target_1.pose.orientation.y = 0;
    target_1.pose.orientation.z = 0;
    target_1.pose.orientation.w = 1;
    target_points.push_back(target_1);



    // 第5个目标点
    geometry_msgs::PoseStamped target_2;
    target_2.pose.position.x = 1.85;
    target_2.pose.position.y = -1.52;
    target_2.pose.position.z = 1.0;
    // //yaw角设置
    target_2.pose.orientation.x = 0;
    target_2.pose.orientation.y = 0;
    target_2.pose.orientation.z = 0;
    target_2.pose.orientation.w = 1;
    target_points.push_back(target_2);
    //动态靶标轨迹中心点
    //降落
    dynamic_point.pose.position.x = 6.55;
    dynamic_point.pose.position.y = 1.3;
    dynamic_point.pose.position.z = 1.0;
    
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
                // current_state = DroneState::TRACKING_WAYPOINT;

                // current_state = DroneState::CROSS_LAND;
                // current_state = DroneState::DECIDekE_DYNAMIC;

                ROS_INFO("TAKE SUCCESS");
                // Ser_pub('U');
            }
            break; 
        case DroneState::DECIDE_TRACK:
            //ego-planner的调试
            decide_track.pose.position.x = 0.1;
            decide_track.pose.position.y = 0;
            decide_track.pose.position.z = 1.0;
            decide_track.pose.orientation.x = 0;
            decide_track.pose.orientation.y = 0;
            decide_track.pose.orientation.z = 0;
            decide_track.pose.orientation.w = 1;
            pos_pub.publish(decide_track);
            if (std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.05 && std::abs(pose_data.pose_local.pose.position.x - 0.1) <0.05)
            {
                current_state = DroneState::TRACKING_WAYPOINT;
                ROS_INFO("OK");
            }
            break;
        case DroneState::DECIDE_DYNAMIC:
            //准备动态靶标
            //yaw角设置
            // position_pub.publish(dynamic_position);
            //测试点
            //正式版
            dynamic_position.pose.position.x =6.48;
            dynamic_position.pose.position.y = -0.3;
            dynamic_position.pose.position.z = 1.2;
            dynamic_position.pose.orientation.x = 0;
            dynamic_position.pose.orientation.y = 0;
            dynamic_position.pose.orientation.z = 0;
            dynamic_position.pose.orientation.w = 1;
            if(ego_contral)
            {
                position_pub.publish(dynamic_position);
                ego_contral = false;
                ROS_INFO("--sending--");
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(dynamic_position);          
            ROS_INFO("---decideing---");
            if(std::abs(pose_data.pose_local.pose.position.x - dynamic_position.pose.position.x)< 0.05 && std::abs(pose_data.pose_local.pose.position.y -dynamic_position.pose.position.y) < 0.05 && std::abs(pose_data.pose_local.pose.position.z - dynamic_position.pose.position.z) < 0.05 )
            {
                // 采样5秒内的delta_x, delta_y
                std::vector<double> delta_x_samples;
                std::vector<double> delta_y_samples;
                ros::Time start_time = ros::Time::now();
                ros::Duration sample_duration(5.0);
                while ((ros::Time::now() - start_time) < sample_duration)
                {
                    double temp_dx, temp_dy, temp_dz;
                    computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy, pose_data.pose_local.pose.position.z, pose_data.pose_local.pose.orientation, temp_dx, temp_dy, temp_dz);
                    delta_x_samples.push_back(temp_dx);
                    delta_y_samples.push_back(temp_dy);
                    ros::Duration(0.05).sleep(); // 20Hz采样
                    ros::spinOnce();
                }
                // 计算中位数 - 先排序后取中间值
                auto median = [](std::vector<double>& v) -> double {
                    if (v.empty()) return 0.0;
                    // 对数据进行排序
                    std::sort(v.begin(), v.end());
                    size_t n = v.size() / 2;
                    if (v.size() % 2 == 0) {
                        // 偶数个元素，取中间两个的平均值
                        return (v[n - 1] + v[n]) / 2.0;
                    } else {
                        // 奇数个元素，取中间值
                        return v[n];
                    }
                };
                double median_dx = median(delta_x_samples);
                double median_dy = median(delta_y_samples);
                Adjust_point.pose.position.x = median_dx + pose_data.pose_local.pose.position.x - 0.2 ;
                Adjust_point.pose.position.y = median_dy + pose_data.pose_local.pose.position.y ;
                Adjust_point.pose.position.z = 0.6;
                Adjust_point.pose.orientation.x = 0;
                Adjust_point.pose.orientation.y = 0;
                Adjust_point.pose.orientation.z = 0;
                Adjust_point.pose.orientation.w = 1;
                droping_flag = false;
                printf("median x:%f \n",median_dx);
                printf("median y:%f \n",median_dy);
                printf("z:%f \n",Adjust_point.pose.position.z);
                current_class.data = image_staff.image_data.detected_class;
                pos_pub.publish(Adjust_point);
                ros::Duration(2.0).sleep();
                ego_contral = true;
                // Ser_pub('C');
                // Ser_pub('U');
                // Ser_pub('P');
                // current_state=DroneState::LAND;
                ROS_INFO("------OK------");
                current_state=DroneState::DYNAMIC_DROP;
                // current_state = DroneState::PREPARE_TUNNEL;
                // current_state = DroneState::FINISH_Dynamic;//进入靶标追踪状态
            }
            // printf("%f\n",quaternionToYaw(pose_data.pose_local.pose.orientation));
            break;
        // case DroneState::DECIDE_DYNAMIC_02:
        //     if (std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x) <0.05 && std::abs(pose_data.pose_local.pose.position.y - Adjust_point.pose.position.y)<0.05)
        //     {
        //         computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
        //         Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x -0.2;
        //         Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
        //         Adjust_point.pose.position.z = 0.35;
        //         Adjust_point.pose.orientation.x = 0;
        //         Adjust_point.pose.orientation.y = 0;
        //         Adjust_point.pose.orientation.z = 0;
        //         Adjust_point.pose.orientation.w = 1;
        //         pos_pub.publish(Adjust_point);
        //         ROS_INFO("---SECOND---");
        //         current_state = DroneState::DYNAMIC_DROP;
        //     }
        //     break;
        case DroneState::DYNAMIC_DROP:
            if(std::abs(pose_data.pose_local.pose.position.z - 0.6) < 0.05)
            {
                judge_start.data = 1;
                judge_start_pub.publish(judge_start);
                if (dynamic_staff.dynamic_judge.data)
                {
                    ROS_INFO("-----DROPING---");
                    Ser_pub('U');
                    current_state = DroneState::HIGHING;
                }
            }
            break;
            
        case DroneState::DEBUG02:
            Debug_point.pose.position.x = 2.0;
            Debug_point.pose.position.y = 0.0;
            Debug_point.pose.position.z = 0.3;
                //yaw角设置
            Debug_point.pose.orientation.x = 0;
            Debug_point.pose.orientation.y = 0;
            Debug_point.pose.orientation.z = 0;
            Debug_point.pose.orientation.w = 1;
            pos_pub.publish(Debug_point);
            if (std::abs(pose_data.pose_local.pose.position.z - 0.3 < 0.05))
            {
                current_state = DroneState::HIGHING;
                Ser_pub('C');
                Ser_pub('U');
                Ser_pub('P');

            }
            
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
            ROS_INFO("--high--");
            if (std::abs(pose_data.pose_local.pose.position.z - 1.0 < 0.05))
            {
                ros::Duration(1.0).sleep();
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
                if(std::abs(pose_data.pose_local.pose.position.z - 1.0) < 0.15)
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
            drop_finish_point.pose.position.x = -2.37;//7.4;
            drop_finish_point.pose.position.y = 7.4;//2.37;
            drop_finish_point.pose.position.z = 1.0;
            drop_finish_point.pose.orientation.x = 0;
            drop_finish_point.pose.orientation.y = 0;
            drop_finish_point.pose.orientation.z = 0;
            drop_finish_point.pose.orientation.w = 1;
            if(ego_contral)
            {
                position_pub.publish(drop_finish_point);
                ego_contral = false;
                ROS_INFO("--sending--");
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(drop_finish_point);
            if (std::abs(pose_data.pose_local.pose.position.x - drop_finish_point.pose.position.x) <0.05 && std::abs(pose_data.pose_local.pose.position.z - drop_finish_point.pose.position.z)<0.05 && std::abs(pose_data.pose_local.pose.position.y - drop_finish_point.pose.position.y)<0.05)
            {
                ROS_INFO("DECIDE CROSS");
                current_state = DroneState::CROSS_LAND;
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
                        else if (target_class == "bunker")
                        {
                            dropped_classes.erase("bunker");
                        }
                        else if (target_class == "bridge")
                        {
                            dropped_classes.erase("bridge");
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
                if(droping_second && getLengthBetweenPoints(pose_data.pose_local.pose.position,Adjust_point.pose.position)<0.15)
                {
                    if (image_staff.image_data.detected_class != "bridge" && image_staff.image_data.detected_class != "car" && image_staff.image_data.detected_class != "bunker" && !((mission_num == 1 && goods_num == 3) || (mission_num == 2 && goods_num == 2) || (mission_num == 3 && goods_num == 1)))                    
                    {
                        current_state = DroneState::TRACKING_WAYPOINT;
                        mission_num+=1;  
                        printf("null or not ");
                        droping_flag = true;
                    }
                    else 
                    {
                        printf("success\n");
                        if(static_cast<int>(Drop_queue.size()) == 3 && second_adjust)
                        {
                            //根据第一个进行调整 :C
                            computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                            Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x+0.2;
                            Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
                            Adjust_point.pose.position.z = 0.3;
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x + 0.2;
                            // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y ;
                            // Adjust_point.pose.position.z =  0.3;
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
                        else if(static_cast<int>(Drop_queue.size()) == 2 && second_adjust)
                        {
                            //根据第二个进行调整 :U
                            ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                            Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x ;
                            Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y +0.2;
                            Adjust_point.pose.position.z = 0.3;
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x -0.2;
                            // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y  ;
                            // Adjust_point.pose.position.z =  0.3;
                            // // Adjust_point.pose.position.z =  0.5;
                            // // //yaw角设置
                            // Adjust_point.pose.orientation.x = 0;
                            // Adjust_point.pose.orientation.y = 0;
                            // Adjust_point.pose.orientation.z = 0;
                            // Adjust_point.pose.orientation.w = 1;
                            pos_pub.publish(Adjust_point);
                            second_adjust = false;
                            droping_second = false;
                            ROS_INFO("2");
                        }
                        else if(static_cast<int>(Drop_queue.size()) == 1 && second_adjust)
                        {
                            //根据第二个进行调整 :U
                            ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                            Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x-0.2 ;
                            Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
                            Adjust_point.pose.position.z = 0.3;
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            // Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x -0.2;
                            // Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y  ;
                            // Adjust_point.pose.position.z =  0.3;
                            // // Adjust_point.pose.position.z =  0.5;
                            // // //yaw角设置
                            // Adjust_point.pose.orientation.x = 0;
                            // Adjust_point.pose.orientation.y = 0;
                            // Adjust_point.pose.orientation.z = 0;
                            // Adjust_point.pose.orientation.w = 1;
                            pos_pub.publish(Adjust_point);
                            second_adjust = false;
                            droping_second = false;
                            ROS_INFO("1");
                        }
                    }
                }
                if(std::abs(pose_data.pose_local.pose.position.z - 0.3) < 0.05 && std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x) <0.03 && std::abs(pose_data.pose_local.pose.position.y - Adjust_point.pose.position.y)<0.03)
                { 

                    ROS_INFO_THROTTLE(1.0, "Data: %s", current_class.data.c_str());
                    if(mission_num ==3)
                    {
                        Ser_pub(Drop_queue.front());
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.0).sleep();
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
                    if(mission_num == 2)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        // goods_num--;   
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.0).sleep();
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
                    if(mission_num == 1)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        // goods_num--;   
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.0).sleep();
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
                    if(current_class.data =="bridge" && mission_num != 4 && mission_num != 3 && mission_num !=2)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        ROS_INFO("Droping Finsh ");
                        ros::Duration(1.0).sleep();
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
                    if(current_class.data == "car" && mission_num != 4 && mission_num != 3  && mission_num !=2)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        // current_class.data = "";
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.0).sleep();
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
                    if(current_class.data == "bunker" && mission_num != 4 && mission_num != 3  && mission_num !=2)
                    {
                        Ser_pub(Drop_queue.front());
                        Drop_queue.pop();
                        // current_class.data = "";
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        second_adjust = true;
                        ros::Duration(1.0).sleep();
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
            finish_Point.pose.position.x =0;
            finish_Point.pose.position.y = 0;
            finish_Point.pose.position.z = 1.0;
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
                current_state = DroneState::LAND;
                //正式状态
                // current_state = DroneState::CROSS_LAND;
                ego_contral = true;
            }           
            break; 

        case DroneState::CROSS_LAND:
            // cross_land_point.pose.position.x = 7.4;
            // cross_land_point.pose.position.y = 2.3;
            // cross_land_point.pose.position.z = 0.6;

            cross_land_point.pose.position.x= drop_finish_point.pose.position.x;
            cross_land_point.pose.position.y= drop_finish_point.pose.position.y;
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


        case DroneState::CHANGE_YAW:
            change_yaw_point.pose.position = cross_land_point.pose.position;
            // //对应角度的四元数
            change_yaw_point.pose.orientation.x = 0;
            change_yaw_point.pose.orientation.y = 0;
            change_yaw_point.pose.orientation.z = 0.7;
            change_yaw_point.pose.orientation.w = -0.7;
            pos_pub.publish(change_yaw_point);
            printf("error_1:%f\n",getLengthBetweenPoints(pose_data.pose_local.pose.position,change_yaw_point.pose.position));
            printf("error_2:%f\n",getAbsYawDifference(pose_data.pose_local.pose.orientation,change_yaw_point.pose.orientation));
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,change_yaw_point.pose.position) < 0.2 && getAbsYawDifference(pose_data.pose_local.pose.orientation,change_yaw_point.pose.orientation) < 0.15)
            {
                ROS_INFO("---CHANGE FINISH---");
                current_state = DroneState::DECIDE_CROSS;
            }
            break;

        case DroneState::DECIDE_CROSS:
            // cross_point.pose.orientation = change_yaw_point.pose.orientation;
            if (cross_judge)
            {
                cross_pub.publish(cross_point);
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(cross_point);
            current_state = DroneState::JUDGE_CROSS;
            // current_state = DroneState::LAND;
            ROS_INFO("-send-");
            // cross_judge = false;
            break;

        case DroneState::JUDGE_CROSS:
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_point.pose.position) < 0.2)
            {
                ROS_INFO("---success---");
                current_state = DroneState::DECIDE_CROSS02;
                // current_state = DroneState::LAND;

            }
            ROS_INFO("---gonging---");
            break;
        


        case DroneState::DECIDE_CROSS02:
            if (cross_judge)
            {
                cross_pub.publish(cross_point_02);
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(cross_point_02);
            current_state = DroneState::JUDGE_CROSS02;
            ROS_INFO("-send-");
            // cross_judge = false;
            break;

        
        case DroneState::JUDGE_CROSS02:
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_point_02.pose.position) < 0.2)
            {
                ROS_INFO("---success---");
                current_state = DroneState::LAND;
            }
            ROS_INFO("---gonging---");
            break;
            
        case DroneState::DECIDE_CROSS03:
            if (cross_judge)
            {
                cross_pub.publish(cross_point_03);
                ros::Duration(1.0).sleep();
            }
            pos_pub.publish(cross_point_03);
            current_state = DroneState::JUDGE_CROSS03;
            ROS_INFO("-send-");
            // cross_judge = false;
            break;

        case DroneState::JUDGE_CROSS03:
            if (getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_point_03.pose.position) < 0.2)
            {
                ROS_INFO("---success---");
                current_state = DroneState::LAND;
            }
            ROS_INFO("---gonging---");
            break;          
        case DroneState::LAND:
                //正式点
                // land_point.pose.position.x = cross_point.pose.position.x;
                // land_point.pose.position.y = cross_point.pose.position.y;
                // land_point.pose.position.z = -0.01;

                //测试点
                land_point.pose.position.x = cross_point_02.pose.position.x;
                land_point.pose.position.y = cross_point_02.pose.position.y;
                land_point.pose.position.z = 0;

                land_point.pose.orientation = cross_point_02.pose.orientation;
                // land_point.pose.orientation.x = 0;
                // land_point.pose.orientation.y = 0;
                // land_point.pose.orientation.z =0;
                // land_point.pose.orientation.w = 1;
                current_state = current_state= DroneState::FINISH;
                pos_pub.publish(land_point);
                ROS_INFO("--LANDING---");
            // if(qr_num.datas[2] == "left")
            // {
            //     ROS_INFO("LAND");
            //     // position_pub.publish(decide_left);
            //     pos_pub.publish(decide_left);
            //     land_point.pose.position.x = decide_left.pose.position.x;
            //     land_point.pose.position.y = decide_left.pose.position.y;
            //     land_point.pose.position.z = 1.0;
            //     current_state= DroneState::FINISH;
            // }
            // if(qr_num.datas[2] == "right")
            // {
            //     ROS_INFO("LAND");
            //     // position_pub.publish(decide_right);
            //     pos_pub.publish(decide_right);
            //     land_point.pose.position.x = decide_right.pose.position.x;
            //     land_point.pose.position.y = decide_right.pose.position.y;
            //     land_point.pose.position.z = 1.0;

            //     current_state= DroneState::FINISH;
            // }
            break;
            
        case DroneState::FINISH:
            // if(std::abs(pose_data.pose_local.pose.position.x - land_point.pose.position.x)<0.05 && std::abs(pose_data.pose_local.pose.position.y - land_point.pose.position.y)<0.05 && std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.05)
            // {
            //     ros::Duration(1.0).sleep();
            //     if(qr_num.datas[2] == "left")
            //     {
            //         ROS_INFO("LAND");
            //         land_point.pose.position.x = decide_left.pose.position.x;
            //         land_point.pose.position.y = decide_left.pose.position.y;
            //         land_point.pose.position.z = 0.0;
            //         land_point.pose.orientation.x = 0;
            //         land_point.pose.orientation.y = 0;
            //         land_point.pose.orientation.z = 0;
            //         land_point.pose.orientation.w = 1; 
            //         pos_pub.publish(land_point);
               
            //     }
            //     if(qr_num.datas[2] == "right")
            //     {
            //         ROS_INFO("LAND");
            //         land_point.pose.position.x = decide_right.pose.position.x;
            //         land_point.pose.position.y = decide_right.pose.position.y;
            //         land_point.pose.position.z = 0.0;
            //         land_point.pose.orientation.x = 0;
            //         land_point.pose.orientation.y = 0;
            //         land_point.pose.orientation.z = 0;
            //         land_point.pose.orientation.w = 1; 
            //         pos_pub.publish(land_point);

            //     }
            //     // land_point.pose.position.x = pose_data.pose_local.pose.position.x;
            //     // land_point.pose.position.y = pose_data.pose_local.pose.position.y;
            //     // land_point.pose.position.z = 0.0;
            //     // pos_pub.publish(land_point);
            // }
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
void MissionFSM::pose_pub(const std::vector<geometry_msgs::PoseStamped>& target_points,int flag)
{
    static bool trj_judge = true;
    static ros::Time last_request = ros::Time::now();  // 确保初始化
    ros::Time current_time = ros::Time::now();
    ros::Duration time_since_last_request = current_time - last_request;
    // 检查 target_points 是否包含足够的点
    if (target_points.size() < 5) {
        ROS_ERROR("target_points size is less than 5. Current size: %zu", target_points.size());
    }
    if (true)
    {       
            last_request = current_time;
            if(trj_judge)
            {
                position_pub.publish(target_points[flag]);
                pos_pub.publish(target_points[flag]);
                // class_staff.classfiy_data.data = "";
                // class_staff.confidence_ = 0;
                trj_judge = false;
                //开启航点的采集
                startDataCollection();
            }
            pos_pub.publish(target_points[flag]);

            if(std::abs(pose_data.pose_local.pose.position.x - target_points[flag].pose.position.x )< 0.1 && std::abs(pose_data.pose_local.pose.position.y - target_points[flag].pose.position.y) < 0.1 && std::abs(pose_data.pose_local.pose.position.z - target_points[flag].pose.position.z)<0.1){  // 使用 flag - 1
                current_state= DroneState::DROPING;
                trj_judge = true;

                //ROS_INFO("SUCCRSS");
            }
    }
}

//识别二维码的第一次目标点
bool MissionFSM::first_pub(const std::vector<geometry_msgs::PoseStamped>& points) {
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
    double x_cam =  ((u -cx) / fx) * depth;
    double y_cam =  -((v -cy) / fy) * depth;
    double z_cam =  -depth;
    //图像到无人机坐标
    delta_x = cos(yaw)*(y_cam + dx) - (-x_cam + dy )*sin(yaw);
    delta_y = cos(yaw)*(-x_cam + dy) + (y_cam + dx)*sin(yaw);
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

// 新增：在飞行过程中采集数据
void MissionFSM::collectFlightData() {
    if (!is_collecting_data) return;
    
    // 检查是否有有效的相机数据
    if ((image_staff.image_data.cx != 0 || image_staff.image_data.cy != 0) && (!image_staff.image_data.detected_class.empty()) && checkWithCount(image_staff.image_data.detected_class))    
    {
        double temp_dx, temp_dy, temp_dz;
        computeAdjustment(image_staff.image_data.cx, image_staff.image_data.cy, 
                         pose_data.pose_local.pose.position.z, 
                         pose_data.pose_local.pose.orientation, 
                         temp_dx, temp_dy, temp_dz);
        
        // 创建数据样本并添加到容器
        FlightDataSample sample;
        sample.tar_x = temp_dx + pose_data.pose_local.pose.position.x;
        sample.tar_y = temp_dy + pose_data.pose_local.pose.position.y;
        sample.cur_class.data = image_staff.image_data.detected_class;
        sample.timestamp = ros::Time::now();
        sample.drone_position = pose_data.pose_local.pose.position;
        
        flight_data_samples.push_back(sample);
        
        // 可选：限制样本数量以避免内存过度使用
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
        current_detected_class != "bridge" && current_detected_class != "tant") {
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
