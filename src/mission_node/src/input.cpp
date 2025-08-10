#include "input.h"
#include <cmath>

// StateSub 类的 feed 函数实现
void StateSub::feed(const mavros_msgs::State::ConstPtr& msg) {
    state_code = *msg;  // 获取状态码
}

// PoseSub 类的 local_pose_cb 函数实现
void PoseSub::local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_local = *msg;  // 直接赋值
    
}
void QrSub::Qr_cb(const std_msgs::String::ConstPtr& msg)
{
    qr_data= *msg;
    boost::split(datas, qr_data.data, boost::is_any_of(","));
    ROS_INFO("Subscriber created!");  // 检查订阅是否成功
}
void ImageSub::Image_cb(const mission_node::Bounding_box::ConstPtr& msg)
{
    image_data = *msg;
}
void ClassifySub::Classify_cb(const mission_node::class_pub::ConstPtr& msg)
{
    class_data = *msg;
    // if(confidence_ < class_data.confidence)
    // {
    //     confidence_ = class_data.confidence;
    //     classfiy_data.data = class_data.classify_class;
    // }

        // 滑动窗口投票法
    const int WINDOW_SIZE = 7;  // 窗口大小
    const double MIN_CONFIDENCE = 0.6;  // 最小置信度阈值
    
    // 只有置信度足够高才加入窗口
    if (class_data.confidence > MIN_CONFIDENCE) {
        
        classification_window.push_back(class_data.classify_class);
        
        // 保持窗口大小
        if (classification_window.size() > WINDOW_SIZE) {
            classification_window.pop_front();
        }
        
        // 投票决定最终类别
        if (classification_window.size() >= 5) {  // 至少5个样本才投票
            std::map<std::string, int> votes;
            for (const auto& cls : classification_window) {
                votes[cls]++;
            }
            
            // 找到票数最多的类别
            auto max_vote = std::max_element(votes.begin(), votes.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; });
            
            // 如果得票数超过一半，则更新分类结果
            if (max_vote->second > classification_window.size() / 2) 
            {
                auto it = string_mapping.find(max_vote->first);
                if (it != string_mapping.end()) 
                {
                    // std::cout << "找到映射: " << max_vote->first << " -> " << it->second << std::endl;
                    classfiy_data.data = it->second;
                }
                else 
                {
                    classfiy_data.data = max_vote->first;
                    // std::cout << "未找到映射: " << max_vote->first << std::endl;
                }
                printf("class:%s \n",classfiy_data.data.c_str());
                // classfiy_data.data = max_vote->first;
            }
        }
    }
}
void CircleSub::circle_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    // circle_pose = *msg;  // 直接赋值
    if(judge_circle_center)
    {
        ring_center_buffer.push_back(msg->pose.position);
        if(ring_center_buffer.size() > 20) {
            // 计算环形中心点的平均位置
            geometry_msgs::Point avg_center;
            avg_center.x = 0;
            avg_center.y = 0; 
            avg_center.z = 0;

            for(const auto& point : ring_center_buffer) 
            {
                avg_center.x += point.x;
                avg_center.y += point.y;
                avg_center.z += point.z;
            }
            avg_center.x /= ring_center_buffer.size();
            avg_center.y /= ring_center_buffer.size();
            avg_center.z /= ring_center_buffer.size();
            circle_pose.pose.position.x = avg_center.x;
            circle_pose.pose.position.y = 1.6;
            circle_pose.pose.position.z = 1.50;
            // ring_center_received = true;
            ring_center_buffer.clear();
            judge_circle_center = false;
    }
}
}
void ObjSub::Obj_posSub(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    obj_position = *msg;
}
void TrajSub::Traj_Sub(const std_msgs::UInt8::ConstPtr& msg)
{
    traj_sub = *msg;
}
void DynamicSub::Dynamic_CB(const std_msgs::Bool::ConstPtr& msg)
{
    dynamic_judge = *msg;
}