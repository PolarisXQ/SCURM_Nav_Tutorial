#include "/home/sentry_ws/src/sentry_waypoint_loader_cpp/include/waypoint_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <iostream>

using namespace sentry_waypoint_loader_cpp;
using namespace std::chrono_literals;

// 构造函数：初始化节点、参数与组件
WaypointLoader::WaypointLoader(const rclcpp::NodeOptions& options) 
    : Node("sentry_waypoint_loader_node", options), current_z_(0.0) {
    RCLCPP_INFO(this->get_logger(), "初始化航点加载器节点...");
    init_parameters();   // 第一步：获取参数
    init_components();   // 第二步：初始化组件
    // 第三步：启动延迟定时器（延迟后开始解析航点）
    start_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(start_delay_),
        std::bind(&WaypointLoader::on_start_timer, this)
    );
}

// 初始化节点参数（从参数服务器或默认值获取）
void WaypointLoader::init_parameters() {
    // 1. 声明参数并设置默认值
    this->declare_parameter("start_delay", 2.0, 
        rcl_interfaces::msg::ParameterDescriptor{}
            .set__description("节点启动后延迟多久开始解析航点（单位：秒）"));
    this->declare_parameter("waypoints_file", 
        std::string(getenv("HOME") + std::string("/sentry_ws/src/sentry_waypoint_loader_cpp/config/waypoints.yaml")),
        rcl_interfaces::msg::ParameterDescriptor{}
            .set__description("获取航点YAML文件的绝对路径"));
    
    // 2. 获取参数值
    this->get_parameter("start_delay", start_delay_);
    this->get_parameter("waypoints_file", waypoints_path_);

    RCLCPP_INFO(this->get_logger(), "参数初始化完成：");
    RCLCPP_INFO(this->get_logger(), "  - 启动延迟：%.1f秒", start_delay_);
    RCLCPP_INFO(this->get_logger(), "  - YAML路径：%s", waypoints_path_.c_str());
}

// 初始化Action客户端、速度发布者等组件
void WaypointLoader::init_components() {
    // 1. 初始化FollowWaypoints Action客户端
    follow_action_client_ = rclcpp_action::create_client<FollowWaypoints>(
        this, "/follow_waypoints");  // 对应Nav2的waypoint_follower动作话题

    // 2. 初始化速度控制发布者（用于Z轴上升）
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);  // 队列大小10，确保消息不丢失

    // 3. 高度控制发布器（带时间戳）
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_stamped", 10);

    // 4. 高度订阅器
    height_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 10,  // 订阅里程计话题
        std::bind(&WaypointLoader::height_callback, this, std::placeholders::_1)
    );

    // 5. 任务执行器+任务注册+任务服务
    task_executor_ = std::make_shared<SimpleWaypointTaskExecutor>();
    // 注册4种任务
    task_executor_->registerTask("ascend_200mm", 
        [this](){ return this->execute_ascend_200mm(); });
    task_executor_->registerTask("ascend_400mm", 
        [this](){ return this->execute_ascend_400mm(); });
    task_executor_->registerTask("delay_descend_200mm", 
        [this](){ return this->execute_delay_descend_200mm(); });
    task_executor_->registerTask("delay_descend_400mm", 
        [this](){ return this->execute_delay_descend_400mm(); });

    RCLCPP_INFO(this->get_logger(), 
        "组件初始化完成：Action客户端、速度发布器、任务执行器已创建");

    // 初始化自定义航点任务插件
    task_plugin_ = std::make_shared<sentry_waypoint_loader_cpp::SentryWaypointTask>();
    RCLCPP_INFO(this->get_logger(), "Sentry航点任务插件实例创建成功");
}

// 解析YAML文件中的所有航点（主航点+过渡航点）
bool WaypointLoader::parse_all_waypoints_from_yaml() {
    // 打开YAML文件
    std::ifstream yaml_file(waypoints_path_);
    if (!yaml_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开YAML文件：%s", waypoints_path_.c_str());
        return false;
    }

    // 加载YAML根节点
    YAML::Node root;
    try {
        root = YAML::Load(yaml_file);
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML解析错误：%s", e.what());
        return false;
    }

    // 读取prepoints（航点ID列表，用于加载所有航点数据，保留此逻辑）
    if (!root["prepoints"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML中缺少prepoints字段（航点ID列表）");
        return false;
    }
    prepoints_ = root["prepoints"].as<std::vector<std::string>>();
    RCLCPP_INFO(this->get_logger(), "读取到%d个航点ID（prepoints）", (int)prepoints_.size());

    // 读取所有航点的位姿数据（主+过渡，保留此逻辑）
    all_wp_map_.clear();  // 清空之前的数据
    for (const auto& wp_id_str : prepoints_) {
        if (!root[wp_id_str]) {
            RCLCPP_WARN(this->get_logger(), "YAML中缺少航点：%s，跳过", wp_id_str.c_str());
            continue;
        }

        // 解析航点位姿
        geometry_msgs::msg::PoseStamped wp;
        try {
            wp.header.frame_id = root[wp_id_str]["header"]["frame_id"].as<std::string>();
            wp.header.stamp = this->get_clock()->now();
            wp.pose.position.x = root[wp_id_str]["pose"]["position"]["x"].as<double>();
            wp.pose.position.y = root[wp_id_str]["pose"]["position"]["y"].as<double>();
            wp.pose.position.z = root[wp_id_str]["pose"]["position"]["z"].as<double>();
            wp.pose.orientation.x = root[wp_id_str]["pose"]["orientation"]["x"].as<double>();
            wp.pose.orientation.y = root[wp_id_str]["pose"]["orientation"]["y"].as<double>();
            wp.pose.orientation.z = root[wp_id_str]["pose"]["orientation"]["z"].as<double>();
            wp.pose.orientation.w = root[wp_id_str]["pose"]["orientation"]["w"].as<double>();

            // 读取 YAML 中的 task 字段（如果存在的话）
            if (root[wp_id_str]["task"]) {
                try {
                    WaypointTaskInfo task_info;
                    // 读取 YAML 中的 action（对应 "ascend" 或 "delayed_descend"）
                    task_info.action = root[wp_id_str]["task"]["action"].as<std::string>();
                    // 读取 YAML 中的 height_mm（对应 200 或 400）
                    task_info.height_mm = root[wp_id_str]["task"]["height_mm"].as<int>();
                    // 关联航点ID和任务信息
                    wp_task_map_[wp_id_str] = task_info;
                    RCLCPP_DEBUG(this->get_logger(), "航点%s绑定任务：action=%s, height_mm=%d",
                        wp_id_str.c_str(), task_info.action.c_str(), task_info.height_mm);
                } catch (const YAML::Exception& e) {
                    RCLCPP_WARN(this->get_logger(), "解析航点%s的task字段失败：%s", wp_id_str.c_str(), e.what());
                    // 即使 task 解析失败，也不中断整体流程，继续解析其他航点
                    continue;
                }
            }

            all_wp_map_[wp_id_str] = wp;
            RCLCPP_DEBUG(this->get_logger(), "解析航点：%s → (X:%.2f, Y:%.2f)", 
                wp_id_str.c_str(), wp.pose.position.x, wp.pose.position.y);
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析航点%s失败：%s", wp_id_str.c_str(), e.what());
            return false;
        }
    }

    // 读取主航点列表（waypoints字段），并新增main_waypoint_ids_存储ID
    if (!root["waypoints"]) {
        RCLCPP_ERROR(this->get_logger(), "YAML中缺少waypoints字段（主航点ID列表）");
        return false;
    }
    std::vector<int> main_wp_ids = root["waypoints"].as<std::vector<int>>();
    waypoints_.clear();         // 清空主航点位姿列表
    main_waypoint_ids_.clear(); // 清空主航点ID列表

    for (int wp_id : main_wp_ids) {
        std::string wp_id_str = std::to_string(wp_id);
        if (all_wp_map_.count(wp_id_str)) {
            main_waypoint_ids_.push_back(wp_id);  // 只添加waypoints中定义的ID
            waypoints_.push_back(all_wp_map_[wp_id_str]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "主航点ID%d在YAML中不存在！", wp_id);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "添加主航点：ID%d → (X:%.2f, Y:%.2f)", 
            wp_id, all_wp_map_[wp_id_str].pose.position.x, all_wp_map_[wp_id_str].pose.position.y);
    }

    if (waypoints_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "主航点列表为空，无法启动导航");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "YAML解析完成：共%d个主航点，%d个总航点", 
        (int)waypoints_.size(), (int)all_wp_map_.size());
    return true;
}


// 判断两个主航点间的关系，匹配过渡点
WaypointLoader::WaypointRelation WaypointLoader::judge_waypoint_relation(int start_id, int end_id) {
    int id_diff = end_id - start_id;

    //左侧以及右侧无法离开梅林限制，直接写出节点id为了增加代码可读性，不建议取模计算
    // 0,3,6,9,12 不能有+1关系（即不能作为start_id且end_id=start_id+1）
    if (id_diff == 1) {
        if (start_id == 0 || start_id == 3 || start_id == 6 || start_id == 9 || start_id == 12) {
            RCLCPP_ERROR(this->get_logger(), "航点%d向左移动掉出梅林，行为禁止", start_id);
            return WaypointRelation::RELATION_ERROR;
        }
    }
    // -2,1,4,7,10 不能有-1关系（即不能作为start_id且end_id=start_id-1）
    else if (id_diff == -1) {
        if (start_id == -2 || start_id == 1 || start_id == 4 || start_id == 7 || start_id == 10) {
            RCLCPP_ERROR(this->get_logger(), "航点%d向右移动掉出梅林，行为禁止", start_id);
            return WaypointRelation::RELATION_ERROR;
        }
    }

    if (id_diff == 3) {
        return WaypointRelation::RELATION_PLUS_3;
    } else if (id_diff == -3) {
        return WaypointRelation::RELATION_MINUS_3;
    } else if (id_diff == 1) {
        return WaypointRelation::RELATION_PLUS_1;
    } else if (id_diff == -1) {
        return WaypointRelation::RELATION_MINUS_1;
    } else {
        RCLCPP_ERROR(this->get_logger(), "航点关系无效：start_id=%d, end_id=%d, 差值=%d", 
            start_id, end_id, id_diff);
        return WaypointRelation::RELATION_ERROR;
    }
}

// 根据主航点关系获取两个过渡点
bool WaypointLoader::get_transition_points(int start_id, int end_id, 
    geometry_msgs::msg::PoseStamped& trans1, geometry_msgs::msg::PoseStamped& trans2) {
    
    // 判断航点关系
    WaypointRelation relation = judge_waypoint_relation(start_id, end_id);
    if (relation == WaypointRelation::RELATION_ERROR) {
        return false;
    }

    // 生成过渡点ID（根据关系匹配_front/_back/_left/_right）
    std::string tp1_id_str, tp2_id_str;
    switch (relation) {
        case WaypointRelation::RELATION_PLUS_3:
            tp1_id_str = std::to_string(start_id) + "_front";
            tp2_id_str = std::to_string(end_id) + "_back";
            break;
        case WaypointRelation::RELATION_MINUS_3:
            tp1_id_str = std::to_string(start_id) + "_back";
            tp2_id_str = std::to_string(end_id) + "_front";
            break;
        case WaypointRelation::RELATION_PLUS_1:
            tp1_id_str = std::to_string(start_id) + "_left";
            tp2_id_str = std::to_string(end_id) + "_right";
            break;
        case WaypointRelation::RELATION_MINUS_1:
            tp1_id_str = std::to_string(start_id) + "_right";
            tp2_id_str = std::to_string(end_id) + "_left";
            break;
        default:
            return false;
    }

    // 从航点地图中获取过渡点位姿
    if (!all_wp_map_.count(tp1_id_str)) {
        RCLCPP_ERROR(this->get_logger(), "过渡点1不存在：%s", tp1_id_str.c_str());
        return false;
    }
    if (!all_wp_map_.count(tp2_id_str)) {
        RCLCPP_ERROR(this->get_logger(), "过渡点2不存在：%s", tp2_id_str.c_str());
        return false;
    }

    trans1 = all_wp_map_[tp1_id_str];
    trans2 = all_wp_map_[tp2_id_str];
    RCLCPP_DEBUG(this->get_logger(), "生成过渡点：%s → %s", 
        tp1_id_str.c_str(), tp2_id_str.c_str());
    return true;
}

// 辅助函数：通过航点索引找到对应的航点ID（如 "-2_front"），效率非常低，未来考虑优化
std::string WaypointLoader::get_wp_id_by_index(uint32_t index) {
    if (index >= full_waypoints_.size()) return "";
    auto& target_pose = full_waypoints_[index].pose;

    // 遍历 all_wp_map_，匹配位姿（x/y/z 误差允许±0.001）
    for (const auto& [wp_id, wp_pose] : all_wp_map_) {
        double x_diff = fabs(wp_pose.pose.position.x - target_pose.position.x);
        double y_diff = fabs(wp_pose.pose.position.y - target_pose.position.y);
        double z_diff = fabs(wp_pose.pose.position.z - target_pose.position.z);
        if (x_diff < 0.001 && y_diff < 0.001 && z_diff < 0.001) {
            return wp_id;
        }
    }
    return "";
}

// 构建完整路径（主航点+过渡点：主→过1→过2→主→...）
bool WaypointLoader::build_full_waypath() {
    full_waypoints_.clear();  // 先清空，避免累积旧数据

    // 主航点数量：由YAML的waypoints字段决定（如[-1,2]则size=2）
    size_t main_count = main_waypoint_ids_.size();
    if (main_count == 0) {
        RCLCPP_ERROR(this->get_logger(), "主航点列表为空，无法构建路径");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "开始构建路径：共%d个主航点", (int)main_count);

    // 遍历主航点，插入过渡点
    for (size_t i = 0; i < main_count - 1; ++i) {
        // 获取当前主航点与下一个主航点的ID
        // 从main_waypoint_ids_中取当前和下一个主航点的ID
        int start_id = main_waypoint_ids_[i];    // 第i个主航点ID
        int end_id = main_waypoint_ids_[i+1];    // 第i+1个主航点ID
        // int start_id = std::stoi(prepoints_[i]);  // prepoints与waypoints顺序一致
        // int end_id = std::stoi(prepoints_[i+1]);
        geometry_msgs::msg::PoseStamped trans1, trans2;

        // 添加当前主航点
        full_waypoints_.push_back(waypoints_[i]);
        // 获取并添加两个过渡点
        if (!get_transition_points(start_id, end_id, trans1, trans2)) {
            RCLCPP_ERROR(this->get_logger(), "主航点%d→%d的过渡点生成失败，中断路径构建", start_id, end_id);
            full_waypoints_.clear();
            return false;
        }
        full_waypoints_.push_back(trans1);  // 过渡点1（执行Z轴上升）
        full_waypoints_.push_back(trans2);  // 过渡点2（执行延迟2s）
    }

    // 添加最后一个主航点
    full_waypoints_.push_back(waypoints_.back());

    // 若最后一个主航点是10、11、12，即R2准备出梅林，额外添加其_front过渡点负责下降
    int last_main_id = main_waypoint_ids_.back();
    if (last_main_id == 10 || last_main_id == 11 || last_main_id == 12) {
        std::string front_id_str = std::to_string(last_main_id) + "_front";
        if (!all_wp_map_.count(front_id_str)) {
            RCLCPP_ERROR(this->get_logger(), "最后一个主航点%d_front过渡航点不存在：%s", last_main_id, front_id_str.c_str());
            full_waypoints_.clear();
            return false;
        }
        // 添加_front过渡点
        full_waypoints_.push_back(all_wp_map_[front_id_str]);
        RCLCPP_INFO(this->get_logger(), "离开梅林航点%d,执行离开操作过渡航点已添加：%s", last_main_id, front_id_str.c_str());
    }

    // 打印完整路径信息
    RCLCPP_INFO(this->get_logger(), "完整路径构建完成：共%d个航点", (int)full_waypoints_.size());
    for (size_t i = 0; i < full_waypoints_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  航点%d：(X:%.2f, Y:%.2f, 帧ID:%s)",
            (int)i,
            full_waypoints_[i].pose.position.x,
            full_waypoints_[i].pose.position.y,
            full_waypoints_[i].header.frame_id.c_str()
        );
    }
    return true;
}

// 启动延迟定时器回调：开始解析航点并启动导航
void WaypointLoader::on_start_timer() {
    RCLCPP_INFO(this->get_logger(), "启动延迟结束，开始解析航点...");
    
    // 解析YAML并构建完整路径
    if (!parse_all_waypoints_from_yaml() || !build_full_waypath()) {
        RCLCPP_ERROR(this->get_logger(), "航点解析或路径构建失败，无法启动导航");
        return;
    }

    if (task_plugin_) {
        // 1. 传递航点ID→任务的映射
        task_plugin_->setTaskMap(wp_task_map_);
        // 2. 传递通过索引查询航点ID的函数
        auto get_id_func = std::bind(&WaypointLoader::get_wp_id_by_index, this, std::placeholders::_1);
        task_plugin_->setGetWpIdFunc(get_id_func);
        RCLCPP_INFO(this->get_logger(), "Sentry航点任务插件数据传递完成（共%d个航点绑定任务）", (int)wp_task_map_.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "插件实例未创建，无法传递数据");
        return;
    }

    // 等待Action服务器就绪
    if (!wait_for_action_server_with_timeout(std::chrono::seconds(10))) {  // 超时10秒
        RCLCPP_ERROR(this->get_logger(), "未连接到FollowWaypoints Action服务器，导航终止");
        return;
    }

    // 发送导航目标
    send_nav_goal();

    // 取消定时器，防止重复触发
    start_timer_->cancel();  
}

// 等待Action服务器（带超时）
bool WaypointLoader::wait_for_action_server_with_timeout(const std::chrono::seconds& timeout) {
    auto start_time = std::chrono::steady_clock::now();
    while (!follow_action_client_->wait_for_action_server(1s)) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "等待FollowWaypoints Action服务器（剩余%.1f秒）...",
            std::chrono::duration<double>(timeout - (std::chrono::steady_clock::now() - start_time)).count());
    }
    RCLCPP_INFO(this->get_logger(), "已连接到FollowWaypoints Action服务器");
    return true;
}

// 发送完整路径导航目标
void WaypointLoader::send_nav_goal() {
    if (full_waypoints_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "完整路径为空，无法发送导航目标");
        return;
    }

    // 构建导航目标消息
    auto goal_msg = FollowWaypoints::Goal();
    goal_msg.poses = full_waypoints_;  // 完整路径（主航点+过渡点）

    // 配置Action回调
    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WaypointLoader::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&WaypointLoader::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&WaypointLoader::result_callback, this, std::placeholders::_1);

    // 发送目标
    follow_action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "已发送导航目标，共%d个航点", (int)full_waypoints_.size());
}

// Action目标响应回调
void WaypointLoader::goal_response_callback(const GoalHandleFollow::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "导航目标被服务器拒绝");
    } else {
        RCLCPP_INFO(this->get_logger(), "导航目标已被服务器接受，开始导航...");
    }
}

// Action反馈回调（实时获取已完成的航点）
void WaypointLoader::feedback_callback(GoalHandleFollow::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    std::lock_guard<std::mutex> lock(waypoint_mutex_);

    // current_waypoint 是当前正在执行的航点索引（从0开始）
    uint32_t current_idx = feedback->current_waypoint;

    // 仅在索引变化时输出日志（避免重复刷屏）
    if (current_idx != last_processed_waypoint_) {
        // 输出当前开始执行的航点信息
        RCLCPP_INFO(this->get_logger(), "开始执行航点%d（共%d个）",
            current_idx, (int)full_waypoints_.size());
        
        // 记录最后一次报告的索引
        last_processed_waypoint_ = current_idx;
    }
}

// void WaypointLoader::feedback_callback(GoalHandleFollow::SharedPtr,
//     const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
//     std::lock_guard<std::mutex> lock(waypoint_mutex_);

//     // current_waypoint 表示“当前正在执行的航点索引”（从0开始）
//     uint32_t current_idx = feedback->current_waypoint;
    
//     // 过滤重复反馈（仅当索引变化时处理）
//     if (current_idx == last_processed_waypoint_) {
//         //【ERROR】此处有问题，索引始终没有变化
//         // RCLCPP_WARN(this->get_logger(), "航点索引%d没有发生变化，耐心等待...", last_processed_waypoint_);
//         return;
//     }

//     // 此处还不能给last_completed_waypoint_赋值，需要利用last_processed_waypoint_来执行任务
//     // 关键步骤1：通过索引找到刚完成的航点的ID（"x_front等形式"）
//     std::string last_processed_wp_id = get_wp_id_by_index(last_processed_waypoint_);
//     if (last_processed_wp_id.empty()) {
//         RCLCPP_WARN(this->get_logger(), "航点索引%d未找到对应的航点ID", last_processed_waypoint_);
//         return;
//     }

//     // 关键步骤2：检查该航点是否有绑定任务
//     if (wp_task_map_.count(last_processed_wp_id) == 0) {
//         RCLCPP_DEBUG(this->get_logger(), "航点%s无绑定任务，继续导航", last_processed_wp_id.c_str());
//         return;
//     }

//     // 关键步骤3：获取任务信息并执行
//     auto& task_info = wp_task_map_[last_processed_wp_id];
//     RCLCPP_INFO(this->get_logger(), "触发航点%s任务：%s，高度%dmm",
//         last_processed_wp_id.c_str(), task_info.action.c_str(), task_info.height_mm);

//     // 根据 YAML 中的 action 和 height_mm 执行对应任务
//     if (task_info.action == "ascend") {
//         if (task_info.height_mm == 200) {
//             execute_ascend_200mm();  // 默认空参数，实际可传递task信息
//         } else if (task_info.height_mm == 400) {
//             execute_ascend_400mm();
//         }
//     } else if (task_info.action == "delayed_descend") {
//         if (task_info.height_mm == 200) {
//             execute_delay_descend_200mm();
//         } else if (task_info.height_mm == 400) {
//             execute_delay_descend_400mm();
//         }
//     }

//     // 更新航点索引，切忌提前更新
//     last_processed_waypoint_ = current_idx;

//     //输出当前准备继续执行的航点信息
//     RCLCPP_INFO(this->get_logger(), "正在执行航点%d（共%d个）",
//         current_idx, (int)full_waypoints_.size() - 1);

    
//     //第一代过渡动作触发逻辑
//     // // 检查索引有效性
//     // if (current_idx >= full_waypoints_.size()) {
//     //     RCLCPP_WARN(this->get_logger(), "航点索引超出范围：%u ≥ %zu",
//     //         current_idx, full_waypoints_.size());
//     //     return;
//     // }
//     // 
//     // // 过渡点1（索引模3=1）：执行Z轴上升（在开始执行该航点时触发）
//     // if (current_idx % 3 == 1 && !rise_triggered_) {
//     //     RCLCPP_INFO(this->get_logger(), "触发过渡点1动作（Z轴上升）");
//     //     execute_transition1_action();
//     //     rise_triggered_ = true;
//     // }
//     // // 过渡点2（索引模3=2）：执行延迟2秒（在开始执行该航点时触发）
//     // else if (current_idx % 3 == 2) {
//     //     RCLCPP_INFO(this->get_logger(), "触发过渡点2动作（延迟2秒）");
//     //     execute_transition2_action();
//     //     rise_triggered_ = false;  // 重置，为下一组过渡点准备
//     // }
// }


// Action结果回调（导航完成/失败）
void WaypointLoader::result_callback(const GoalHandleFollow::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result->missed_waypoints.empty()) {
                RCLCPP_INFO(this->get_logger(), "导航成功完成！所有航点均到达");
            } else {
                RCLCPP_WARN(this->get_logger(), "导航完成，但有 %zu 个航点未到达",
                           result.result->missed_waypoints.size());
                for (const auto& missed_idx : result.result->missed_waypoints) {
                    RCLCPP_WARN(this->get_logger(), "未到达航点索引：%u", missed_idx);
                }
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "导航被中止！");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "导航被取消！");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "导航结果未知（错误码：%d）", (int)result.code);
            break;
    }

    // 重置状态
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
}

//第一代过渡动作实现逻辑
// // 过渡点1动作：Z轴上升1秒（1.0m/s）
// void WaypointLoader::execute_transition1_action() {
//     geometry_msgs::msg::Twist cmd;
//     cmd.linear.z = 1.0;  // 上升速度

//     // 连续发布10次（10×100ms=1秒），确保执行器收到
//     for (int i = 0; i < 10; ++i) {
//         cmd_vel_pub_->publish(cmd);
//         rclcpp::sleep_for(100ms);
//     }

//     // 发布停止指令（冗余5次）
//     cmd.linear.z = 0.0;
//     for (int i = 0; i < 5; ++i) {
//         cmd_vel_pub_->publish(cmd);
//         rclcpp::sleep_for(100ms);
//     }
//     RCLCPP_INFO(this->get_logger(), "过渡点1动作完成（Z轴上升1秒）");
// }

// // 过渡点2动作：延迟2秒
// void WaypointLoader::execute_transition2_action() {
//     rclcpp::sleep_for(2000ms);  // 延迟2秒
//     RCLCPP_INFO(this->get_logger(), "过渡点2动作完成（延迟2秒）");
// }

// 高度订阅回调：更新当前高度
void WaypointLoader::height_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 从里程计消息中提取base_link的Z轴高度
    current_z_ = msg->pose.pose.position.z;
    RCLCPP_DEBUG(this->get_logger(), "当前高度：%.3fm", current_z_);
}

// 任务实现：上升200mm
bool WaypointLoader::execute_ascend_200mm() {
    RCLCPP_INFO(this->get_logger(), "开始执行上升200mm任务（当前高度：%.3fm）", current_z_);
    double target_z = current_z_ + 0.25;  // 目标高度=当前+0.25m
    double tolerance = 0.1;             // 允许±10cm误差
    double ascend_speed = 0.15;           // 上升速度（0.15m/s，可调整）

    // 发布上升指令
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.twist.linear.z = ascend_speed;
    vel_pub_->publish(cmd);

    // 等待到达目标高度
    rclcpp::Rate rate(10);  // 10Hz循环检查
    while (rclcpp::ok()) {
        if (current_z_ >= target_z - tolerance) {
            break;
        }
        RCLCPP_DEBUG(this->get_logger(), "上升中：当前%.3fm / 目标%.3fm", current_z_, target_z);
        rate.sleep();
    }

    // 停止上升
    cmd.twist.linear.z = 0.0;
    cmd.header.stamp = this->get_clock()->now();
    vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "上升200mm任务完成（最终高度：%.3fm）", current_z_);
    return true;
}

// 任务实现：上升400mm
bool WaypointLoader::execute_ascend_400mm() {
    RCLCPP_INFO(this->get_logger(), "开始执行上升400mm任务（当前高度：%.3fm）", current_z_);
    double target_z = current_z_ + 0.45;  // 目标高度+0.4m
    double tolerance = 0.1;
    double ascend_speed = 0.15;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.twist.linear.z = ascend_speed;
    vel_pub_->publish(cmd);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        if (current_z_ >= target_z - tolerance) {
            break;
        }
        RCLCPP_DEBUG(this->get_logger(), "上升中：当前%.3fm / 目标%.3fm", current_z_, target_z);
        rate.sleep();
    }

    cmd.twist.linear.z = 0.0;
    cmd.header.stamp = this->get_clock()->now();
    vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "上升400mm任务完成（最终高度：%.3fm）", current_z_);
    return true;
}

// 任务实现：延时降落400mm
bool WaypointLoader::execute_delay_descend_200mm() {
    RCLCPP_INFO(this->get_logger(), "开始执行延时降落200mm任务（当前高度：%.3fm）", current_z_);
    double target_z = current_z_ - 0.2;  // 目标高度-0.2m
    double tolerance = 0.02;
    double descend_speed = -0.1;         // 降落速度（负号表示向下）

    // 第一步：延时2秒
    RCLCPP_INFO(this->get_logger(), "进入延时2秒...");
    rclcpp::sleep_for(2s);

    // 第二步：发布降落指令
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.twist.linear.z = descend_speed;
    vel_pub_->publish(cmd);

    // 第三步：等待到达目标高度
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        if (current_z_ <= target_z + tolerance) {
            break;
        }
        RCLCPP_DEBUG(this->get_logger(), "降落中：当前%.3fm / 目标%.3fm", current_z_, target_z);
        rate.sleep();
    }

    // 停止降落
    cmd.twist.linear.z = 0.0;
    cmd.header.stamp = this->get_clock()->now();
    vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "延时降落200mm任务完成（最终高度：%.3fm）", current_z_);
    return true;
}

// 任务实现：延时降落400mm
bool WaypointLoader::execute_delay_descend_400mm() {
    RCLCPP_INFO(this->get_logger(), "开始执行延时降落400mm任务（当前高度：%.3fm）", current_z_);
    double target_z = current_z_ - 0.4;  // 目标高度-0.4m
    double tolerance = 0.02;
    double descend_speed = -0.1;

    // 延时2秒
    RCLCPP_INFO(this->get_logger(), "进入延时2秒...");
    rclcpp::sleep_for(2s);

    // 发布降落指令
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->get_clock()->now();
    cmd.twist.linear.z = descend_speed;
    vel_pub_->publish(cmd);

    // 等待到达目标高度
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        if (current_z_ <= target_z + tolerance) {
            break;
        }
        RCLCPP_DEBUG(this->get_logger(), "降落中：当前%.3fm / 目标%.3fm", current_z_, target_z);
        rate.sleep();
    }

    // 停止降落
    cmd.twist.linear.z = 0.0;
    cmd.header.stamp = this->get_clock()->now();
    vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "延时降落400mm任务完成（最终高度：%.3fm）", current_z_);
    return true;
}

// 添加标准main函数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sentry_waypoint_loader_cpp::WaypointLoader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}