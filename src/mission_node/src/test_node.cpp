#include "input.h"
#include "MissionFsm.h"
// #include "input.h"
#include <quadrotor_msgs/TakeoffLand.h>
#include "cmath"


const double fx = 1092.34009;  // 焦距（像素）
const double fy = 1088.58832;
const double cx = 657.880369;
const double cy = 361.681183;
const double dx = 0;    // 相机在无人机机体坐标系的偏移（x: 右，y: 前，z: 上）
const double dy = 0.21;
const double dz = 0.0;

//调整量
double delta_x, delta_y, delta_z;
geometry_msgs::Quaternion debug_quat;

//微调位置
geometry_msgs::PoseStamped Adjust_point;

MissionFSM::MissionFSM() : rate(20.0) {
    // 其他初始化
    current_state = DroneState::INIT;
    // current_state = DroneState::DECIDE_CROSS;
    Drop_queue.push('s');
    Drop_queue.push('S');
    mission_num = 0;
    droping_flag = true ;
    droping_second = false;
    yaw_judge = false;
    cross_judge = false;
    droping_i = 300; 
    goods_num = 2;
    
    //current_state = DroneState::DROPING;
    //测试点
    Debug_point.pose.position.x = 6.2;
    Debug_point.pose.position.y = 0.0;
    Debug_point.pose.position.z = 1.0;
    //
    //降落点初始化
    //right 
    decide_right.pose.position.x = 0.0;
    decide_right.pose.position.y = -1.56;
    decide_right.pose.position.z = 1.0;
    decide_right.pose.orientation.x = 0;
    decide_right.pose.orientation.y = 0;
    decide_right.pose.orientation.z = 0;
    decide_right.pose.orientation.w = 1;
    //left
    decide_left.pose.position.x = 0;
    decide_left.pose.position.y = 1.61;
    decide_left.pose.position.z = 1.0;
    decide_left.pose.orientation.x = 0;
    decide_left.pose.orientation.y = 0;
    decide_left.pose.orientation.z = 0;
    decide_left.pose.orientation.w = 1;
    //起飞后的第一个点，识别二维码

    detect_point.pose.position.x = 1.9;
    detect_point.pose.position.y = 0.0;
    detect_point.pose.position.z = 1.0;

    //由特殊把到目标点的避障点
    geometry_msgs::PoseStamped first_1;
    first_1.pose.position.x = 6.0;
    first_1.pose.position.y = -1.0;
    first_1.pose.position.z = 1.0;
        //yaw角设置
    first_1.pose.orientation.x = 0;
    first_1.pose.orientation.y = 0;
    first_1.pose.orientation.z = 0;
    first_1.pose.orientation.w = 1;
    first_points.push_back(first_1);
    // 第二个目标点
    geometry_msgs::PoseStamped first_2;
    first_2.pose.position.x = 3.65;
    first_2.pose.position.y = -1.0;
    first_2.pose.position.z = 1.0;
    first_2.pose.orientation.x = 0;
    first_2.pose.orientation.y = 0;
    first_2.pose.orientation.z = 0;
    first_2.pose.orientation.w = 1;
    first_points.push_back(first_2);
    // //第三个点
    // geometry_msgs::PoseStamped first_3;
    // first_3.pose.position.x = 5.0;
    // first_3.pose.position.y = 0.0;
    // first_3.pose.position.z = 1.0;
    // first_points.push_back(first_3);
    // //第四个点
    // geometry_msgs::PoseStamped first_4;
    // first_4.pose.position.x = 3.5;
    // first_4.pose.position.y = 0.0;
    // first_4.pose.position.z = 1.0;
    // first_points.push_back(first_4);
    //投货的目标位置

    // 第1个目标点
    geometry_msgs::PoseStamped target_3;
    target_3.pose.position.x = 1.95;
    target_3.pose.position.y = 1.5;
    target_3.pose.position.z = 1.0;
    //yaw角设置1
    target_3.pose.orientation.x = 0;
    target_3.pose.orientation.y = 0;
    target_3.pose.orientation.z = 0;
    target_3.pose.orientation.w = 1;

    target_points.push_back(target_3);

    // 第2个目标点
    geometry_msgs::PoseStamped target_4;
    target_4.pose.position.x = 3.7;
    target_4.pose.position.y = 1.5;
    target_4.pose.position.z = 1.0;

    target_4.pose.orientation.x = 0;
    target_4.pose.orientation.y = 0;
    target_4.pose.orientation.z = 0;
    target_4.pose.orientation.w = 1;
    target_points.push_back(target_4);
       // 第3个目标点
    geometry_msgs::PoseStamped target_5;
    target_5.pose.position.x = 6.2;
    target_5.pose.position.y = 0.73;
    target_5.pose.position.z = 1.0;
    //yaw角设置
    target_5.pose.orientation.x = 0;
    target_5.pose.orientation.y = 0;
    target_5.pose.orientation.z = 0;
    target_5.pose.orientation.w = 1;
    target_points.push_back(target_5);

    // 第4个目标点
    geometry_msgs::PoseStamped target_1;
    target_1.pose.position.x = 3.65;
    target_1.pose.position.y = -1.67;
    target_1.pose.position.z = 1.0;
    //yaw角设置
    target_1.pose.orientation.x = 0;
    target_1.pose.orientation.y = 0;
    target_1.pose.orientation.z = 0;
    target_1.pose.orientation.w = 1;
    target_points.push_back(target_1);
    // 第5个目标点
    geometry_msgs::PoseStamped target_2;
    target_2.pose.position.x = 1.84;
    target_2.pose.position.y = -1.61;
    target_2.pose.position.z = 1.0;
    // //yaw角设置
    target_2.pose.orientation.x = 0;
    target_2.pose.orientation.y = 0;
    target_2.pose.orientation.z = 0;
    target_2.pose.orientation.w = 1;
    target_points.push_back(target_2);
    
    //测试点集，飞正方形
    geometry_msgs::PoseStamped test_point_1 ;
    test_point_1.pose.position.x = 1.0;
    test_point_1.pose.position.y = 0;
    test_point_1.pose.position.z = 1.0;
    test_points.push_back(test_point_1);

    geometry_msgs::PoseStamped test_point_2 ;
    test_point_2.pose.position.x = 1.0;
    test_point_2.pose.position.y = 1.0;
    test_point_2.pose.position.z = 1.0;
    test_points.push_back(test_point_2);

    geometry_msgs::PoseStamped test_point_3;
    test_point_3.pose.position.x = 0.0;
    test_point_3.pose.position.y = 1.0;
    test_point_3.pose.position.z = 1.0;
    test_points.push_back(test_point_3);

    geometry_msgs::PoseStamped test_point_4 ;
    test_point_4.pose.position.x = 0.0;
    test_point_4.pose.position.y = 0.0;
    test_point_4.pose.position.z = 1.0;
    test_points.push_back(test_point_4);
}

void MissionFSM::process() 
{
    
    // ros::Time time = ros::Time::now();
    //ros::Rate r(1.0 / 3.0);
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
            if(std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.1)
            {
                // current_state = DroneState::DETECT_DATA;
                // current_state = DroneState::DEBUG;
                current_state = DroneState::DECIDE_CROSS;
                ROS_INFO("TAKE SUCCESS");

            }
            break; 
        case DroneState::DEBUG:
            circle_staff.judge_circle_center = true;
            printf("data:%f\n",circle_staff.circle_pose.pose.position.x);
            // position_pub.publish(Debug_point);
            // pos_pub.publish(Debug_point);
            // if(getLengthBetweenPoints(pose_data.pose_local.pose.position,Debug_point.pose.position) <0.05)
            // {
            //     ROS_INFO("Success");
            //     printf("%f \n",getLengthBetweenPoints(pose_data.pose_local.pose.position,Debug_point.pose.position));
            //     current_state = DroneState::FINISH;
            //     land_point.pose.position.x = Debug_point.pose.position.x;
            //     land_point.pose.position.y = Debug_point.pose.position.y;
            //     land_point.pose.position.z = 0.0;
            //     pos_pub.publish(land_point);
            // }
            // else{
            //     ROS_INFO("ERROR");
            //     printf("%f \n",getLengthBetweenPoints(pose_data.pose_local.pose.position,Debug_point.pose.position));  
            // }
            break;
        case DroneState::DETECT_DATA:
            ROS_INFO("-----detecting----");
            pos_pub.publish(detect_point);
            //视觉部分，判断是否读取到信息并进行赋值
            if (!qr_num.qr_data.data.empty())
            {  
                current_state = DroneState::TRACKING_WAYPOINT;
                // class_staff.classfiy_data.data = "";
                // class_staff.confidence_ = 0;
                std::cout << "第 " << 1 << " 个数据: " << qr_num.datas[0] << std::endl;
                std::cout << "第 " << 2 << " 个数据: " << qr_num.datas[1] << std::endl;
                std::cout << "第 " << 3 << " 个数据: " << qr_num.datas[2] << std::endl;
            }
            break;
        case DroneState::DECIDE_CROSS:
            //debug 穿环
            cross_01.pose.position.x = pose_data.pose_local.pose.position.x;
            cross_01.pose.position.y = pose_data.pose_local.pose.position.y;
            cross_01.pose.position.z = 1.5;
            //yaw角设置
            cross_01.pose.orientation.x = 0;
            cross_01.pose.orientation.y = 0;
            cross_01.pose.orientation.z = 0;
            cross_01.pose.orientation.w = 1;
            pos_pub.publish(cross_01);
            ROS_INFO("debug");
            if(std::abs(pose_data.pose_local.pose.position.x - cross_01.pose.position.x)< 0.05 && std::abs(pose_data.pose_local.pose.position.y -cross_01.pose.position.y) < 0.05 && std::abs(pose_data.pose_local.pose.position.z - cross_01.pose.position.z) < 0.05 )
            {
                ros::Duration(1.0).sleep();
                ROS_INFO("------OK------");
                current_state = DroneState::PREPARE_TUNNEL;
            }
            // printf("%f\n",quaternionToYaw(pose_data.pose_local.pose.orientation));
            break;
        case DroneState::DECIDE_DROPING:
            //进行障碍物避障
            ROS_INFO("DECIDE TO ZHANGAI WU");
            if(first_pub(first_points))
            {
                ROS_INFO("FINISH");
                // class_staff.classfiy_data.data = "";
                // class_staff.confidence_ = 0;
                current_state = DroneState::TRACKING_WAYPOINT;
            }
            // position_pub.publish(Debug_point);
            // pos_pub.publish(Debug_point);
            // if(std::abs(pose_data.pose_local.pose.position.x - Debug_point.pose.position.x )< 0.05 && std::abs(pose_data.pose_local.pose.position.y - Debug_point.pose.position.y) < 0.05 && std::abs(pose_data.pose_local.pose.position.z - Debug_point.pose.position.z)<0.1)
            // {
            //     ROS_INFO("SUCESS");
            //     //ros::Duration(3.0).sleep();
            //     //current_state = DroneState::TRACKING_WAYPOINT ;//TAKEOFF
            //     current_state = DroneState::DROPING;
            // }

            break;
        case DroneState::TRACKING_WAYPOINT:
            ROS_INFO(" TO TARGET POINT ");
            printf("goods_num:%d",goods_num);
            if(goods_num == 0 && mission_num > 2 )
            {
                current_state = DroneState::LAND ;//TAKEOFF
            }
            else if(mission_num < 5) //&& goods_num!=0)
            {
                ROS_INFO("CONINTIUTE ");
                if(std::abs(pose_data.pose_local.pose.position.z - 1.0) < 0.05)
                {
                    ROS_INFO("----high----");
                    pose_pub(target_points,mission_num);
                    printf("num:%d \n",mission_num);
                }
            }
            if(mission_num == 5)
            {
                current_state = DroneState::LAND ;//TAKEOFF
                ROS_INFO("finish");
            }
            break;  
        case DroneState::DROPING:
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            if(!image_staff.image_data.detected_class.empty() || !class_staff.class_data.classify_class.empty())
            {
                // //判断是否进行降落
                // classify_debug.classify_class = class_staff.class_data.classify_class;
                // classify_debug.confidence = class_staff.class_data.confidence;
                if(droping_flag )
                {
                    //dropping_flag代表微调检测不会进入循环
                    //dropping_seconf代表微调完成后不会进入

                    //computeAdjustment(514,109,1.2 ,debug_quat,delta_x, delta_y,delta_z);
                    computeAdjustment(image_staff.image_data.cx,image_staff.image_data.cy,pose_data.pose_local.pose.position.z ,pose_data.pose_local.pose.orientation,delta_x, delta_y,delta_z);
                    Adjust_point.pose.position.x = delta_x + pose_data.pose_local.pose.position.x ;
                    Adjust_point.pose.position.y = delta_y + pose_data.pose_local.pose.position.y ;
                    Adjust_point.pose.position.z =  pose_data.pose_local.pose.position.z ;
                    droping_flag = false;
                    printf("x:%f \n",delta_x);
                    printf("y:%f \n",delta_y);
                    printf("z:%f \n",Adjust_point.pose.position.z);
                    pos_pub.publish(Adjust_point);
                    if(mission_num == 2)
                    {
                        ros::Duration(1.0).sleep();
                    }
                    droping_second = true;
                    current_class.data =  class_staff.classfiy_data.data;
                }
                if(droping_second && (std::abs(pose_data.pose_local.pose.position.x - Adjust_point.pose.position.x ) < 0.05 ) && std::abs(pose_data.pose_local.pose.position.y -Adjust_point.pose.position.y) < 0.05)
                {
                    if(mission_num!=2 && !(current_class.data ==qr_num.datas[1] || current_class.data ==qr_num.datas[0]) && !((mission_num ==3 && goods_num == 2)  || (mission_num==4 && goods_num ==1)))
                    {
                        current_state = DroneState::TRACKING_WAYPOINT;
                        mission_num+=1;  
                        printf("null or not ");
                        droping_flag = true;
                    }
                    else 
                    {
                        printf("success\n");
                        printf("class: %s\n",current_class.data.c_str());
                        if(mission_num == 2)
                        {
                            Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x - 0.1;
                            Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y  ;
                            Adjust_point.pose.position.z =  0.5;
                            // //yaw角设置
                            Adjust_point.pose.orientation.x = 0;
                            Adjust_point.pose.orientation.y = 0;
                            Adjust_point.pose.orientation.z = 0;
                            Adjust_point.pose.orientation.w = 1;
                            pos_pub.publish(Adjust_point);
                            droping_second = false;
                            ROS_INFO("1");
                        }
                        else
                        {
                            if(static_cast<int>(Drop_queue.size()) == 2)
                            {
                                ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                                Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x +0.1;
                                Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y - 0.1 ;
                                Adjust_point.pose.position.z =  0.5;
                                // //yaw角设置
                                Adjust_point.pose.orientation.x = 0;
                                Adjust_point.pose.orientation.y = 0;
                                Adjust_point.pose.orientation.z = 0;
                                Adjust_point.pose.orientation.w = 1;
                                pos_pub.publish(Adjust_point);
                                droping_second = false;
                                ROS_INFO("2");
                            }
                            else if(static_cast<int>(Drop_queue.size()) == 1)
                            {
                                ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                                Adjust_point.pose.position.x =  pose_data.pose_local.pose.position.x +0.1;
                                Adjust_point.pose.position.y =  pose_data.pose_local.pose.position.y + 0.1 ;
                                Adjust_point.pose.position.z =  0.5;
                                Adjust_point.pose.position.z =  0.5;
                                // //yaw角设置
                                Adjust_point.pose.orientation.x = 0;
                                Adjust_point.pose.orientation.y = 0;
                                Adjust_point.pose.orientation.z = 0;
                                Adjust_point.pose.orientation.w = 1;
                                pos_pub.publish(Adjust_point);
                                droping_second = false;
                                ROS_INFO("3");
                            }
                            else
                            {
                                ROS_INFO("Drop_queue size: %zu", Drop_queue.size());
                            }

                        }

                    }
                }
                if(std::abs(pose_data.pose_local.pose.position.z - 0.5) < 0.05 )
                {
                    if(mission_num == 2)
                    {
                        Ser_pub(Big_drop);
                        ROS_INFO("Droping Finsh ");
                        droping_flag =  true;
                        droping_second = true;
                        ros::Duration(1.0).sleep();
                        // hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                        // hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                        // hight_point.pose.position.z = 1.0;
                        // pos_pub.publish(hight_point);
                        // current_state = DroneState::TRACKING_WAYPOINT;
                        //current_state = DroneState::DECIDE_CROSS
                        while(!first_pub(first_points))
                        {
                            ros::spinOnce();
                            // ROS_INFO("NOT ARRIVE");
                        }
                        mission_num+=1;
                        current_state = DroneState::TRACKING_WAYPOINT;
                        ROS_INFO("OKOKOK");
                        // mission_num+=1;
                    } 
                    else
                    {
                        ROS_INFO_THROTTLE(1.0, "Data: %s", current_class.data.c_str());
                        if(mission_num ==4)
                        {
                            Ser_pub(Drop_queue.front());
                            ROS_INFO("Droping Finsh ");
                            droping_flag =  true;
                            droping_second = true;
                            ros::Duration(1.0).sleep();
                            current_state = DroneState::TRACKING_WAYPOINT;
                            hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                            hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                            hight_point.pose.position.z = 1.0;
                            pos_pub.publish(hight_point);
                            mission_num+=1;
                        }
                        if(mission_num == 3)
                        {
                            Ser_pub(Drop_queue.front());
                            Drop_queue.pop();
                            ROS_INFO("Droping Finsh ");
                            droping_flag =  true;
                            goods_num--;   
                            droping_second = true;
                            ros::Duration(1.0).sleep();
                            current_state = DroneState::TRACKING_WAYPOINT;
                            hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                            hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                            hight_point.pose.position.z = 1.0;
                            pos_pub.publish(hight_point);
                            mission_num+=1;
                            goods_num--;
                        }
                        if(current_class.data == qr_num.datas[0])
                        {
                            qr_num.datas[0] = "";
                            Ser_pub(Drop_queue.front());
                            Drop_queue.pop();
                            ROS_INFO("Droping Finsh ");
                            ros::Duration(1.0).sleep();
                            droping_flag =  true;
                            droping_second = true;
                            current_state = DroneState::TRACKING_WAYPOINT;
                            hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                            hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                            hight_point.pose.position.z = 1.0;
                            pos_pub.publish(hight_point);
                            mission_num+=1;
                            goods_num--;
                        }
                        if(current_class.data == qr_num.datas[1])
                        {
                            qr_num.datas[1] = "";
                            Ser_pub(Drop_queue.front());
                            Drop_queue.pop();
                            ROS_INFO("Droping Finsh ");
                            droping_flag =  true;
                            droping_second = true;
                            ros::Duration(1.0).sleep();
                            current_state = DroneState::TRACKING_WAYPOINT;
                            hight_point.pose.position.x = pose_data.pose_local.pose.position.x;
                            hight_point.pose.position.y = pose_data.pose_local.pose.position.y;
                            hight_point.pose.position.z = 1.0;
                            pos_pub.publish(hight_point);
                            mission_num+=1;
                            goods_num--;
                        }
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
                     
        case DroneState::PREPARE_TUNNEL:
            change_yaw.pose.position.x = pose_data.pose_local.pose.position.x;
            change_yaw.pose.position.y = pose_data.pose_local.pose.position.y;
            change_yaw.pose.position.z = 1.5;
            change_yaw.pose.orientation.x = 0;
            change_yaw.pose.orientation.y = 0;
            change_yaw.pose.orientation.z = 0.7;
            change_yaw.pose.orientation.w = -0.7;     
            pos_pub.publish(change_yaw);
            ROS_INFO("CHANGE FINISH");
            current_state= DroneState::TRACKING_TUNNEL;
            break;
            
        case DroneState::TRACKING_TUNNEL:
            
            if(!yaw_judge && std::abs(quaternionToYaw(pose_data.pose_local.pose.orientation) - quaternionToYaw(change_yaw.pose.orientation)) < 0.08)
            {
                ROS_INFO("FINISH  OK");
                circle_staff.judge_circle_center = true;
                if(circle_staff.circle_pose.pose.position.x!=0)
                {
                    ros::Duration(1.0).sleep();
                    cross_circle.pose.position.x = circle_staff.circle_pose.pose.position.x;
                    cross_circle.pose.position.y = pose_data.pose_local.pose.position.y;
                    cross_circle.pose.position.z = 1.50;
                    cross_circle.pose.orientation.x = 0;
                    cross_circle.pose.orientation.y = 0;
                    cross_circle.pose.orientation.z = 0.7;
                    cross_circle.pose.orientation.w = -0.7; 
                    ROS_INFO("---SUCCESS---");
                    pos_pub.publish(cross_circle);
                    ros::Duration(1.0).sleep();
                    yaw_judge = true;
                }
                ROS_INFO("----CHANGE FINISH-----");
            }
            if(getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_circle.pose.position) < 0.05 )
            {
                ros::Duration(1.0).sleep();
                if( circle_staff.circle_pose.pose.position.x >2 ||  circle_staff.circle_pose.pose.position.x <0)
                {
                    circle_staff.circle_pose.pose.position.x = 0;
                }
                cross_circle.pose.position.x = circle_staff.circle_pose.pose.position.x;
                cross_circle.pose.position.y = -1.8;
                cross_circle.pose.position.z = 1.50;
                cross_circle.pose.orientation.x = 0;
                cross_circle.pose.orientation.y = 0;
                cross_circle.pose.orientation.z = 0.7;
                cross_circle.pose.orientation.w = -0.7; 
                pos_pub.publish(cross_circle);
                ROS_INFO("------CROSS-------");
                cross_judge = true;
            }
            if(cross_judge && getLengthBetweenPoints(pose_data.pose_local.pose.position,cross_circle.pose.position)<0.05)
            {
                ROS_INFO("----OKOK---");
                land_point.pose.position.x = pose_data.pose_local.pose.position.x;
                land_point.pose.position.y = pose_data.pose_local.pose.position.y;
                land_point.pose.position.z = 0.0;
                current_state = current_state= DroneState::FINISH;
                pos_pub.publish(land_point);
                //current_state = DroneState::TRACKING_WAYPOINT;
            }

            break;
        
        case DroneState::LAND:
            if(qr_num.datas[2] == "left")
            {
                ROS_INFO("LAND");
                // position_pub.publish(decide_left);
                pos_pub.publish(decide_left);
                land_point.pose.position.x = decide_left.pose.position.x;
                land_point.pose.position.y = decide_left.pose.position.y;
                land_point.pose.position.z = 1.0;
                current_state= DroneState::FINISH;
            }
            if(qr_num.datas[2] == "right")
            {
                ROS_INFO("LAND");
                // position_pub.publish(decide_right);
                pos_pub.publish(decide_right);
                land_point.pose.position.x = decide_right.pose.position.x;
                land_point.pose.position.y = decide_right.pose.position.y;
                land_point.pose.position.z = 1.0;

                current_state= DroneState::FINISH;
            }
            break;
            
        case DroneState::FINISH:
            if(std::abs(pose_data.pose_local.pose.position.x - land_point.pose.position.x)<0.05 && std::abs(pose_data.pose_local.pose.position.y - land_point.pose.position.y)<0.05 && std::abs(pose_data.pose_local.pose.position.z - 1.0)<0.05)
            {
                ros::Duration(1.0).sleep();
                if(qr_num.datas[2] == "left")
                {
                    ROS_INFO("LAND");
                    land_point.pose.position.x = decide_left.pose.position.x;
                    land_point.pose.position.y = decide_left.pose.position.y;
                    land_point.pose.position.z = 0.0;
                    land_point.pose.orientation.x = 0;
                    land_point.pose.orientation.y = 0;
                    land_point.pose.orientation.z = 0;
                    land_point.pose.orientation.w = 1; 
                    pos_pub.publish(land_point);
               
                }
                if(qr_num.datas[2] == "right")
                {
                    ROS_INFO("LAND");
                    land_point.pose.position.x = decide_right.pose.position.x;
                    land_point.pose.position.y = decide_right.pose.position.y;
                    land_point.pose.position.z = 0.0;
                    land_point.pose.orientation.x = 0;
                    land_point.pose.orientation.y = 0;
                    land_point.pose.orientation.z = 0;
                    land_point.pose.orientation.w = 1; 
                    pos_pub.publish(land_point);

                }
                // land_point.pose.position.x = pose_data.pose_local.pose.position.x;
                // land_point.pose.position.y = pose_data.pose_local.pose.position.y;
                // land_point.pose.position.z = 0.0;
                // pos_pub.publish(land_point);
            }
            if(std::abs(pose_data.pose_local.pose.position.z - 0.0)<0.1)
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
                //position_pub.publish(target_points[flag]);
                pos_pub.publish(target_points[flag]);
                // class_staff.classfiy_data.data = "";
                // class_staff.confidence_ = 0;
                trj_judge = false;
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
                //position_pub.publish(first_points[0]);
                pos_pub.publish(points[0]);
                break;
            case 2:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.05 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.05 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.05)
                {  // 使用 flag - 1
                    flag = 3;
                    //position_pub.publish(first_points[1]);
                    pos_pub.publish(points[1]);
                    ROS_INFO("------FIRST-----");
                }
                break;
            case 3:
                if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.05 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.05 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.05)
                {  // 使用 flag - 1
                    // flag = 4;
                    //position_pub.publish(first_points[2]);
                    // pos_pub.publish(points[2]);
                    judge = true;
                    ROS_INFO("FINSH");
                }
                break;
            // case 4:
            //     if (std::abs(pose_data.pose_local.pose.position.x - points[flag - 2].pose.position.x )< 0.1 && std::abs(pose_data.pose_local.pose.position.y - points[flag - 2].pose.position.y )< 0.1 && std::abs(pose_data.pose_local.pose.position.z - points[flag - 2].pose.position.z)<0.1)
            //     {  // 使用 flag - 1
            //         pos_pub.publish(points[3]);

            //     }
            //     break;
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