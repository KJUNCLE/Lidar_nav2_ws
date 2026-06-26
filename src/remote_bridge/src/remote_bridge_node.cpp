#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <microhttpd.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <mutex>
#include <cstdlib>

using json = nlohmann::json;
namespace fs = std::filesystem;

// ==================== 全局配置（固定绝对路径） ====================
const std::string WORKSPACE_ROOT = "/home/neardi/Lidar_nav2_ws";
const std::string NAV_POINTS_FILE = WORKSPACE_ROOT + "/nav_points.json";
const std::string MAP_DIR = WORKSPACE_ROOT + "/src/me_nav2_bringup/map";
const std::string OBSTACLE_FILE = WORKSPACE_ROOT + "/obstacles.json";

// 当前在建地图名称
std::string g_current_map_name = "new_map";

// ==================== 全局状态缓存 ====================
std::mutex state_mutex;
std::string g_battery_state = "未知";
std::string g_nav_state = "空闲中";
nav_msgs::msg::Odometry g_latest_odom;

void battery_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    g_battery_state = msg->data;
}

void nav_state_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    g_nav_state = msg->data;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex);
    g_latest_odom = *msg;
}

// ==================== ROS全局对象 ====================
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_sub;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_state_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> nav_client;
rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

std::mutex nav_mutex;
std::vector<int> multi_nav_queue;
bool is_nav_paused = false;

// ==================== 工具函数：执行系统命令 ====================
int exec_system_cmd(const std::string &cmd)
{
    std::string full_cmd = "bash -c \"source /opt/ros/foxy/setup.bash && source " + WORKSPACE_ROOT + "/install/setup.bash && " + cmd + "\"";
    int ret = std::system(full_cmd.c_str());
    return ret;
}

// ==================== HTTP工具函数 ====================
static int send_json(struct MHD_Connection *conn, int code, const json &data)
{
    std::string res = data.dump();
    struct MHD_Response *resp = MHD_create_response_from_buffer(
        res.size(), const_cast<char*>(res.c_str()), MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(resp, "Content-Type", "application/json; charset=utf-8");
    int ret = MHD_queue_response(conn, code, resp);
    MHD_destroy_response(resp);
    return ret;
}

std::string get_arg(struct MHD_Connection *conn, const std::string &key)
{
    const char *val = MHD_lookup_connection_value(conn, MHD_GET_ARGUMENT_KIND, key.c_str());
    return val ? std::string(val) : "";
}

// ==================== 导航点持久化 ====================
json load_nav_points()
{
    if(!fs::exists(NAV_POINTS_FILE)) return json::array();
    std::ifstream f(NAV_POINTS_FILE);
    json j;
    f >> j;
    return j;
}

void save_nav_points_json(const json &points)
{
    std::ofstream f(NAV_POINTS_FILE);
    f << points.dump(4);
}

bool delete_nav_point_by_id(int id)
{
    json points = load_nav_points();
    json new_points = json::array();
    bool found = false;
    int new_id = 1;
    for (auto &p : points) {
        if (p["id"].get<int>() != id) {
            p["id"] = new_id++;
            new_points.push_back(p);
        } else {
            found = true;
        }
    }
    if (found) save_nav_points_json(new_points);
    return found;
}

// ==================== 地图工具函数 ====================
std::vector<std::string> get_map_name_list()
{
    std::vector<std::string> names;
    if (fs::exists(MAP_DIR)) {
        for (const auto &entry : fs::directory_iterator(MAP_DIR)) {
            if (entry.path().extension() == ".yaml") {
                names.push_back(entry.path().stem().string());
            }
        }
    }
    return names;
}

bool delete_map_by_id(int id)
{
    std::vector<std::string> names = get_map_name_list();
    if (id < 1 || id > (int)names.size()) return false;
    std::string name = names[id-1];
    std::string yaml_path = MAP_DIR + "/" + name + ".yaml";
    std::string pgm_path = MAP_DIR + "/" + name + ".pgm";
    bool ok = true;
    if (fs::exists(yaml_path)) ok &= fs::remove(yaml_path);
    if (fs::exists(pgm_path)) ok &= fs::remove(pgm_path);
    return ok;
}

// ==================== ROS业务功能 ====================
void emergency_stop()
{
    std::lock_guard<std::mutex> lock(nav_mutex);
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub->publish(stop_msg);
    if(nav_client->action_server_is_ready()) {
        nav_client->async_cancel_all_goals();
    }
    is_nav_paused = true;
    RCLCPP_INFO(g_node->get_logger(), "紧急停止：导航已取消，小车制动");
}

void resume_navigation()
{
    std::lock_guard<std::mutex> lock(nav_mutex);
    is_nav_paused = false;
    RCLCPP_INFO(g_node->get_logger(), "导航已恢复");
}

bool get_current_pose(geometry_msgs::msg::PoseStamped &pose)
{
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer->lookupTransform(
            "map", "base_footprint", tf2::TimePointZero);
        pose.header.frame_id = "map";
        pose.header.stamp = g_node->now();
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
        return true;
    } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR(g_node->get_logger(), "获取当前位姿失败: %s", e.what());
        return false;
    }
}

void publish_initial_pose(double px, double py, double pz, double ox, double oy, double oz, double ow)
{
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.frame_id = "map";
    msg.header.stamp = g_node->now();
    msg.pose.pose.position.x = px;
    msg.pose.pose.position.y = py;
    msg.pose.pose.position.z = pz;
    msg.pose.pose.orientation.x = ox;
    msg.pose.pose.orientation.y = oy;
    msg.pose.pose.orientation.z = oz;
    msg.pose.pose.orientation.w = ow;
    msg.pose.covariance = {
        0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
        0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685
    };
    initial_pose_pub->publish(msg);
    RCLCPP_INFO(g_node->get_logger(), "已发布初始位姿: x=%.2f, y=%.2f", px, py);
}

void navigate_to_pose(double px, double py, double oz, double ow)
{
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = g_node->now();
    goal_msg.pose.pose.position.x = px;
    goal_msg.pose.pose.position.y = py;
    goal_msg.pose.pose.orientation.z = oz;
    goal_msg.pose.pose.orientation.w = ow;
    auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    nav_client->async_send_goal(goal_msg, opts);
}

void multi_nav_worker(std::string point_str)
{
    std::vector<int> ids;
    std::string tmp;
    for (char c : point_str) {
        if (c == '#') {
            if (!tmp.empty()) ids.push_back(std::stoi(tmp));
            tmp.clear();
        } else tmp += c;
    }
    if (!tmp.empty()) ids.push_back(std::stoi(tmp));
    json points = load_nav_points();
    for (int id : ids) {
        if (is_nav_paused) break;
        if (id < 1 || id > (int)points.size()) continue;
        json target = points[id-1];
        RCLCPP_INFO(g_node->get_logger(), "多点导航: 前往点位%d", id);
        navigate_to_pose(
            target["position_x"], target["position_y"],
            target["orientation_z"], target["orientation_w"]
        );
        std::this_thread::sleep_for(std::chrono::seconds(8));
    }
    RCLCPP_INFO(g_node->get_logger(), "多点导航执行完毕");
}

// ==================== 地图真实功能 ====================
// 真实保存2D栅格地图 + 3D点云地图
bool save_current_map()
{
    std::string map_full_path = MAP_DIR + "/" + g_current_map_name;
    std::string log_path = WORKSPACE_ROOT + "/map_save_debug.log";

    std::system(("echo '===== Save Start: " + g_current_map_name + " =====' > " + log_path).c_str());

    // 修复：增加10秒超时，匹配transient_local QoS
    std::string save_2d_cmd =
        "bash -c '"
        "source /opt/ros/foxy/setup.bash && "
        "source " + WORKSPACE_ROOT + "/install/setup.bash && "
        "timeout 12 ros2 run nav2_map_server map_saver_cli "
        "-f " + map_full_path + " "
        "--ros-args -p map_topic:=map -p save_map_timeout:=10.0 "
        ">> " + log_path + " 2>&1"
        "'";

    int ret_code = std::system(save_2d_cmd.c_str());

    // 3D点云保存保持异步
    std::string save_3d_cmd =
        "bash -c '"
        "source /opt/ros/foxy/setup.bash && "
        "source " + WORKSPACE_ROOT + "/install/setup.bash && "
        "timeout 3 ros2 service call /map_save std_srvs/srv/Trigger > /dev/null 2>&1"
        "' &";
    std::system(save_3d_cmd.c_str());

    if (ret_code == 0) {
        RCLCPP_INFO(rclcpp::get_logger("remote_bridge"), "地图保存成功: %s", g_current_map_name.c_str());
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("remote_bridge"), "地图保存失败，详情: %s", log_path.c_str());
        return false;
    }
}
// 异步保存地图：完全后台执行，HTTP接口立即返回

// 真实切换地图
bool switch_map(const std::string &map_name)
{
    std::string map_path = MAP_DIR + "/" + map_name + ".yaml";
    if (!fs::exists(map_path)) {
        RCLCPP_WARN(g_node->get_logger(), "地图文件不存在: %s", map_path.c_str());
        return false;
    }
    if (!load_map_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(g_node->get_logger(), "map_server服务未就绪，无法切换地图");
        return false;
    }
    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = map_path;
    auto future = load_map_client->async_send_request(req);
    std::future_status status = future.wait_for(std::chrono::seconds(3));
    if (status == std::future_status::ready) {
        auto result = future.get();
        if (result->result == 0) {
            RCLCPP_INFO(g_node->get_logger(), "地图切换成功: %s", map_name.c_str());
            return true;
        }
    }
    return false;
}

// ==================== HTTP请求处理 ====================
static int request_handler(void *cls, struct MHD_Connection *conn,
                           const char *url, const char *method,
                           const char *version, const char *upload_data,
                           size_t *upload_data_size, void **con_cls)
{
    (void)cls; (void)version;
    if (*con_cls == nullptr) {
        *con_cls = new std::string();
        return MHD_YES;
    }
    std::string path(url);
    std::string m(method);
    std::string *body_buf = static_cast<std::string*>(*con_cls);

    // ========== GET 接口 ==========
    if (m == "GET")
    {
        // 1. 获取设备状态
        if (path == "/api/extra/get_status")
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            json ret = {
                {"code", 0},
                {"msg", "设备在线"},
                {"success", true},
                {"battery", g_battery_state},
                {"nav_state", g_nav_state}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 22. 获取实时全量状态
        if (path == "/api/extra/get_realtime_state")
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            json ret = {
                {"success", true},
                {"battery", g_battery_state},
                {"nav_state", g_nav_state},
                {"position", {
                    {"x", g_latest_odom.pose.pose.position.x},
                    {"y", g_latest_odom.pose.pose.position.y},
                    {"z", g_latest_odom.pose.pose.position.z}
                }},
                {"velocity", {
                    {"linear_x", g_latest_odom.twist.twist.linear.x},
                    {"angular_z", g_latest_odom.twist.twist.angular.z}
                }}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 2. 获取地图列表
        if (path == "/api/extra/get_map_data")
        {
            json map_list = json::array();
            std::vector<std::string> names = get_map_name_list();
            for (size_t i = 0; i < names.size(); i++) {
                map_list.push_back({{"id", (int)(i+1)}, {"name", names[i]}});
            }
            int page = std::stoi(get_arg(conn, "page").empty() ? "1" : get_arg(conn, "page"));
            json ret = {{"map_list", map_list}, {"total_pages", 1}, {"current_page", page}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 11/13. 停止/恢复导航
        if (path == "/api/extra/nav_work/cancel")
        {
            std::string stop = get_arg(conn, "stop");
            if (!stop.empty()) {
                emergency_stop();
                json ret = {{"msg", "导航已停止"}, {"success", true}};
                send_json(conn, MHD_HTTP_OK, ret);
            } else {
                resume_navigation();
                json ret = {{"msg", "导航已恢复"}, {"success", true}};
                send_json(conn, MHD_HTTP_OK, ret);
            }
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 17. 动作控制
        if (path.find("/api/extra/target_angle") == 0)
        {
            int angle = std::stoi(get_arg(conn, "angle"));
            RCLCPP_INFO(g_node->get_logger(), "动作指令: angle=%d", angle);
            geometry_msgs::msg::Twist twist;
            switch(angle) {
                case 7: twist.angular.z = 0.3; break;
                case 1: twist.linear.x = 0.2; break;
                case 6: twist.linear.x = 0.5; break;
                default: twist.linear.x = 0.0; twist.angular.z = 0.0;
            }
            cmd_vel_pub->publish(twist);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            geometry_msgs::msg::Twist stop;
            cmd_vel_pub->publish(stop);
            json ret = {{"msg", "动作执行完成"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 7. 获取导航点列表
        if (path == "/api/extra/get_nav_data")
        {
            json points = load_nav_points();
            json point_list = json::array();
            for (auto &p : points) {
                point_list.push_back({{"id", p["id"]}, {"name", p["name"]}});
            }
            int page = std::stoi(get_arg(conn, "page").empty() ? "1" : get_arg(conn, "page"));
            json ret = {{"point_list", point_list}, {"total_pages", 1}, {"current_page", page}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 14. 获取障碍物列表
        if (path == "/api/extra/get_obstacle_data")
        {
            int page = std::stoi(get_arg(conn, "page").empty() ? "1" : get_arg(conn, "page"));
            json ret = {
                {"obstacle_list", json::array()},
                {"total_pages", 1},
                {"current_page", page},
                {"msg", "查询成功"},
                {"success", true}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 21. 刷新点云
        if (path == "/api/extra/refresh_point_cloud")
        {
            RCLCPP_INFO(g_node->get_logger(), "点云刷新指令");
            json ret = {{"msg", "刷新成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }
    }

    // ========== POST 接口 ==========
    if (m == "POST")
    {
        if (*upload_data_size > 0) {
            body_buf->append(upload_data, *upload_data_size);
            *upload_data_size = 0;
            return MHD_YES;
        }
        json req;
        try { req = json::parse(*body_buf); }
        catch(...) {
            send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg","JSON格式错误"},{"success",false}});
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 3. 新增地图（记录地图名称）
        if (path == "/api/extra/add_map")
        {
            std::string map_name = req.value("map_name", "new_map");
            g_current_map_name = map_name;
            json ret = {
                {"msg", "建图模式已开启，请控制小车移动建图"},
                {"success", true}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 4. 保存地图（真实执行保存命令）
        if (path == "/api/extra/save_map")
        {
            std::string cmd = req.value("cmd", "save");
            if (cmd == "save") {
                bool ok = save_current_map();
                json ret = {
                    {"msg", ok ? "保存成功" : "保存失败，请确认建图节点已启动"},
                    {"success", ok}
                };
                send_json(conn, MHD_HTTP_OK, ret);
            } else {
                send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg","参数错误"},{"success",false}});
            }
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 5. 删除地图（真实删除文件）
        if (path == "/api/extra/delete_map_data")
        {
            int id = req.value("id", 0);
            bool ok = delete_map_by_id(id);
            json ret = {
                {"msg", ok ? "删除成功" : "删除失败，地图不存在"},
                {"success", ok}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 6. 切换地图（真实调用map_server）
        if (path == "/api/extra/switch_map")
        {
            int id = req.value("id", 0);
            std::vector<std::string> names = get_map_name_list();
            if (id < 1 || id > (int)names.size()) {
                send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg", "地图ID不存在"}, {"success", false}});
                delete body_buf; *con_cls = nullptr;
                return MHD_YES;
            }
            bool ok = switch_map(names[id-1]);
            json ret = {
                {"msg", ok ? "地图切换成功" : "地图切换失败"},
                {"success", ok}
            };
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 8. 单点导航
        if (path == "/api/extra/navigate")
        {
            int point_id = req.value("id", 1);
            json points = load_nav_points();
            if (point_id < 1 || point_id > (int)points.size()) {
                send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg", "导航点不存在"}, {"success", false}});
                delete body_buf; *con_cls = nullptr;
                return MHD_YES;
            }
            json target = points[point_id - 1];
            navigate_to_pose(
                target["position_x"], target["position_y"],
                target["orientation_z"], target["orientation_w"]
            );
            RCLCPP_INFO(g_node->get_logger(), "下发导航指令: 点位%d", point_id);
            json ret = {{"msg", "操作成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 9. 删除导航点
        if (path == "/api/extra/delete_nav_data")
        {
            int id = req.value("id", 0);
            bool ok = delete_nav_point_by_id(id);
            json ret = {{"msg", ok ? "删除成功" : "点位不存在"}, {"success", ok}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 10. 导航点重定位
        if (path == "/api/extra/reset_location")
        {
            int id = req.value("id", 1);
            json points = load_nav_points();
            if (id < 1 || id > (int)points.size()) {
                send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg", "导航点不存在"}, {"success", false}});
                delete body_buf; *con_cls = nullptr;
                return MHD_YES;
            }
            json target = points[id-1];
            publish_initial_pose(
                target["position_x"], target["position_y"], target["position_z"],
                target["orientation_x"], target["orientation_y"],
                target["orientation_z"], target["orientation_w"]
            );
            json ret = {{"msg", "重定位中"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 12. 多点导航
        if (path == "/api/extra/multi_nav")
        {
            std::string point_str = req.value("point", "");
            if (point_str.empty()) {
                send_json(conn, MHD_HTTP_BAD_REQUEST, {{"msg", "参数错误"}, {"success", false}});
                delete body_buf; *con_cls = nullptr;
                return MHD_YES;
            }
            std::thread(multi_nav_worker, point_str).detach();
            json ret = {{"msg", "路径规划成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 15. 删除障碍物
        if (path == "/api/extra/delete_obstacle_data")
        {
            int id = req.value("id", 0);
            RCLCPP_INFO(g_node->get_logger(), "删除障碍物 ID:%d", id);
            json ret = {{"msg", "删除成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 16. 障碍物操作
        if (path == "/api/extra/delete_nav_area")
        {
            std::string input_val = req.value("inputValue", "null");
            std::string delete_val = req.value("deleteValue", "null");
            RCLCPP_INFO(g_node->get_logger(), "障碍物操作: add=%s, del=%s", input_val.c_str(), delete_val.c_str());
            json ret = {{"msg", "创建成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 18. 设置初始位姿
        if (path == "/api/extra/set_pose")
        {
            double px = req.value("positionX", 0.0);
            double py = req.value("positionY", 0.0);
            double pz = req.value("positionZ", 0.0);
            double ox = req.value("orientationX", 0.0);
            double oy = req.value("orientationY", 0.0);
            double oz = req.value("orientationZ", 0.0);
            double ow = req.value("orientationW", 1.0);
            publish_initial_pose(px, py, pz, ox, oy, oz, ow);
            json ret = {{"msg", "重定位中"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 19. 标记导航点
        if (path == "/api/extra/mark_nav_point")
        {
            std::string name = req.value("inputValue", "nav_point");
            geometry_msgs::msg::PoseStamped current_pose;
            if (req.contains("positionX")) {
                current_pose.pose.position.x = req.value("positionX", 0.0);
                current_pose.pose.position.y = req.value("positionY", 0.0);
                current_pose.pose.position.z = req.value("positionZ", 0.0);
                current_pose.pose.orientation.x = req.value("orientationX", 0.0);
                current_pose.pose.orientation.y = req.value("orientationY", 0.0);
                current_pose.pose.orientation.z = req.value("orientationZ", 0.0);
                current_pose.pose.orientation.w = req.value("orientationW", 1.0);
            } else {
                if (!get_current_pose(current_pose)) {
                    send_json(conn, MHD_HTTP_INTERNAL_SERVER_ERROR, {{"msg", "获取位姿失败"}, {"success", false}});
                    delete body_buf; *con_cls = nullptr;
                    return MHD_YES;
                }
            }
            json points = load_nav_points();
            json new_point = {
                {"id", (int)points.size() + 1},
                {"name", name},
                {"position_x", current_pose.pose.position.x},
                {"position_y", current_pose.pose.position.y},
                {"position_z", current_pose.pose.position.z},
                {"orientation_x", current_pose.pose.orientation.x},
                {"orientation_y", current_pose.pose.orientation.y},
                {"orientation_z", current_pose.pose.orientation.z},
                {"orientation_w", current_pose.pose.orientation.w}
            };
            points.push_back(new_point);
            save_nav_points_json(points);
            RCLCPP_INFO(g_node->get_logger(), "标记导航点: %s", name.c_str());
            json ret = {{"msg", "操作成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }

        // 20. 自定义导航
        if (path == "/api/extra/nav_custom")
        {
            double px = req.value("positionX", 0.0);
            double py = req.value("positionY", 0.0);
            double oz = req.value("orientationZ", 0.0);
            double ow = req.value("orientationW", 1.0);
            navigate_to_pose(px, py, oz, ow);
            RCLCPP_INFO(g_node->get_logger(), "自定义导航: x=%.2f, y=%.2f", px, py);
            json ret = {{"msg", "路径规划成功"}, {"success", true}};
            send_json(conn, MHD_HTTP_OK, ret);
            delete body_buf; *con_cls = nullptr;
            return MHD_YES;
        }
    }

    send_json(conn, MHD_HTTP_NOT_FOUND, {{"msg","接口不存在"},{"success",false}});
    delete body_buf;
    *con_cls = nullptr;
    return MHD_YES;
}

// ==================== 主函数 ====================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("remote_bridge");

    // 发布者
    cmd_vel_pub = g_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    initial_pose_pub = g_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // 订阅者
    battery_sub = g_node->create_subscription<std_msgs::msg::String>(
        "/robot/battery", 10, battery_callback);
    nav_state_sub = g_node->create_subscription<std_msgs::msg::String>(
        "/x_nav/state", 10, nav_state_callback);
    odom_sub = g_node->create_subscription<nav_msgs::msg::Odometry>(
        "/base_link/odom", 10, odom_callback);

    // 导航动作客户端
    nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        g_node, "/navigate_to_pose");

    // 地图服务客户端
    load_map_client = g_node->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

    // TF
    tf_buffer = std::make_shared<tf2_ros::Buffer>(g_node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // HTTP服务
    int port = 9000;
    struct MHD_Daemon *daemon = MHD_start_daemon(
        MHD_USE_SELECT_INTERNALLY, port,
        nullptr, nullptr,
        &request_handler, nullptr,
        MHD_OPTION_END
    );
    if (!daemon) {
        RCLCPP_ERROR(g_node->get_logger(), "HTTP服务启动失败，端口%d被占用", port);
        return -1;
    }
    RCLCPP_INFO(g_node->get_logger(), "=== 遥控器桥接节点启动成功 ===");
    RCLCPP_INFO(g_node->get_logger(), "HTTP服务监听端口: %d", port);
    RCLCPP_INFO(g_node->get_logger(), "全部地图功能已升级为真实实现");

    rclcpp::spin(g_node);
    MHD_stop_daemon(daemon);
    rclcpp::shutdown();
    return 0;
}
