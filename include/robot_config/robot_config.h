/*************************************************************************
        > File Name: robot_config.h
        > Author: ma6174
        > Mail: ma6174@163.com
        > Created Time: 2018年08月21日 星期二 10时58分57秒
 ************************************************************************/
#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>

#include <iostream>
#include <fstream>
#include <ctime>
#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>
#include <pwd.h>
#include <vector>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <cti_msgs/msg/robot_param_config.hpp>
#include <cti_msgs/msg/map_data_config.hpp>
#include <cti_msgs/msg/scene_change.hpp>
#include <cti_msgs/msg/vehicle_model.hpp>
#include <cti_msgs/msg/robot_version_display.hpp>
#include <cti_msgs/msg/all_sensor_status.hpp>
#include <cti_msgs/msg/sensor_state.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <cti_msgs/msg/robot_localizer_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <cti_msgs/msg/box_state.hpp>
#include <cti_msgs/msg/tab_state.hpp>
#include <cti_msgs/msg/data.hpp>
#include <cti_msgs/msg/data_array.hpp>

#include <cti_msgs/msg/error_status_array.hpp>
#include <cti_msgs/msg/error_status.hpp>

template <typename T>
T getParam(const YAML::Node &node, const std::string &name,
           const T &defaultValue) {
  T v;
  try {
    v = node[name].as<T>();
  } catch (const std::exception &e) {
    std::cout << "getParam[exception] " << e.what() << std::endl;
    v = defaultValue;
  }
  return v;
}

struct StaticTfState {
  std::string frame_id{""};
  std::string child_frame_id{""};
  double translation_x{0};
  double translation_y{0};
  double translation_z{0};
  double rotation_x{0};
  double rotation_y{0};
  double rotation_z{0};
  double rotation_w{0};
};

struct TopicState {
  std::string name;
  std::string topic;
  std::string type;
  float timeout{0};
  bool enable{false};
  int mul{0};
};

enum StateType {
  ERROR = -1,    //错误，开机未读取到数据
  NORMAL = 0,    //正常
  OVERTIME = 1,  //超时
};

class StateBase {
  struct StateTimer {
    explicit StateTimer(rclcpp::Node *node_) : node__(node_) {
      // node__ = rclcpp::Node::make_shared("ljy");
    }
    // rclcpp::Time start_time = clock.now();

    void set() {
      state = true;
      start_timer = node__->now().seconds();
    }

    [[nodiscard]] double getDuration(double now) const {
      return now - start_timer;
    }
    bool state{false};
    double start_timer{0};
    // std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Node *node__;
  };

 public:
  explicit StateBase(rclcpp::Node *node)
      : node_(node), timer_(node) {}
  virtual ~StateBase() {}
  virtual void init() = 0;
  virtual int getState() = 0;
  virtual std::string getTopic() = 0;
  virtual std::string getName() = 0;
  virtual std::string getType() = 0;
  virtual float getTimeout() = 0;
  virtual bool getEnable() = 0;
  virtual int getMul() = 0;
  virtual void monitor_exist_lidar(const std::string &str) = 0;

 protected:
  rclcpp::Node *node_;
  StateTimer timer_;
};

template <class T>
class SensorState : public StateBase {
  using DataTypeSharedPtr = typename T::SharedPtr;
  using SubscriptionType = typename rclcpp::Subscription<T>::SharedPtr;

 public:
  SensorState(rclcpp::Node *node, TopicState &state, std::string topic_config_path)
      : StateBase(node), data_(state), topic_config_path_(topic_config_path) {
    init();
  }
  ~SensorState() override = default;

 public:
  virtual void init() override;
  virtual int getState() override;
  virtual std::string getTopic() override;
  virtual std::string getName() override;
  std::string getType() override;
  virtual float getTimeout() override;
  virtual bool getEnable() override;
  virtual int getMul() override;
  virtual void monitor_exist_lidar(const std::string &str) override;
  void callback(const DataTypeSharedPtr msg);

 private:
  SubscriptionType sub_;
  TopicState data_;
  std::string topic_config_path_;
};

class RobotConfig : public rclcpp::Node {
 public:
  RobotConfig();
  ~RobotConfig() override;
  void init();

 private:
  void publishSensorState();
  void load_env_param();
  void getTopicStateParams(std::vector<TopicState> &topicStates);
  void publishMapconfig(const std::string &env);
  void pubErrorMsg(const std::string &modular_name, const uint &error_code,
                   const uint &error_level, const std::string &error_msg);
  void monitor_release_data();
  void makeUltStaticTransforms();

  //回调函数声明
  void timercallback();
  void robotVersionCallback(
      const cti_msgs::msg::RobotVersionDisplay::SharedPtr msg);

  void error_status_Callback(const cti_msgs::msg::ErrorStatus::SharedPtr msg);

  void localizerStateCallback(
      const cti_msgs::msg::RobotLocalizerState::SharedPtr msg);

  void robotEnvCallback(const std_msgs::msg::String::SharedPtr msg);
  void getStaticTfParams(const std::string &path);
  void declareVehicleConfigParams();
  void pubParameterServer();
  void parameterServerCallback(const std_msgs::msg::String::SharedPtr msg);

 private:
  //订阅
  rclcpp::Subscription<cti_msgs::msg::RobotVersionDisplay>::SharedPtr
      robot_version_sub_;
  rclcpp::Subscription<cti_msgs::msg::RobotLocalizerState>::SharedPtr
      localizer_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_env_sub_;
  rclcpp::Subscription<cti_msgs::msg::ErrorStatus>::SharedPtr sub_error_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parameter_server_sub;
  

  //发布
  rclcpp::Publisher<cti_msgs::msg::MapDataConfig>::SharedPtr map_data_pub_;
  rclcpp::Publisher<cti_msgs::msg::RobotParamConfig>::SharedPtr robot_parm_pub_;
  rclcpp::Publisher<cti_msgs::msg::RobotVersionDisplay>::SharedPtr
      robot_version_pub_;
  rclcpp::Publisher<cti_msgs::msg::BoxState>::SharedPtr sensor_state_pub_;
  rclcpp::Publisher<cti_msgs::msg::ErrorStatusArray>::SharedPtr
      pub_errorArray_publisher;
  rclcpp::Publisher<cti_msgs::msg::ErrorStatus>::SharedPtr pub_error_msg;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameter_server_pub_;

  //定时器
  rclcpp::TimerBase::SharedPtr timer_;
  //系统参数
  cti_msgs::msg::RobotVersionDisplay robot_version_;
  cti_msgs::msg::ErrorStatusArray error_msg_array;
  std::string cti_map_path_;
  std::string cti_vmap_path_;
  std::string robot_config_file_;
  std::string run_env_;
  int localizer_state_;
  bool release_error_msg{false};
  double time_monitor_err{0};
  std::vector<std::shared_ptr<StateBase>> sensor_states_;
  std::string cti_file_path_;
  std::string cti_topic_config_path_;
  std::string cti_vehicle_config_file_path_;
  std::string cti_static_tf_file_path_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
  std::vector<StaticTfState> staticTfStates_;

  //参数服务
  Json::Value parameter_server_;
};
#endif


