#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "robot_config/robot_config.h"

//--------------------------------------
using std::placeholders::_1;

template <class T>
void SensorState<T>::init() {
  if (!data_.topic.empty()) {
    sub_ = node_->template create_subscription<T>(
        data_.topic, rclcpp::QoS{1}.best_effort(),
        std::bind(&SensorState::callback, this, std::placeholders::_1));
  }
}

template <class T>
int SensorState<T>::getState() {
  int stat = StateType::ERROR;
  if (data_.enable) {
    if (timer_.state) {
      if (timer_.getDuration(node_->now().seconds()) > data_.timeout) {
        stat = StateType::OVERTIME;
      } else {
        stat = StateType::NORMAL;
      }
    }
  } else {
    stat = StateType::NORMAL;
  }
  return stat;
}

template <class T>
std::string SensorState<T>::getTopic() {
  return data_.topic;
}

template <class T>
std::string SensorState<T>::getName() {
  return data_.name;
}

template <class T>
std::string SensorState<T>::getType() {
  return data_.type;
}

template <class T>
float SensorState<T>::getTimeout() {
  return data_.timeout;
}

template <class T>
bool SensorState<T>::getEnable() {
  return data_.enable;
}

template <class T>
int SensorState<T>::getMul() {
  return data_.mul;
}

template <class T>
void SensorState<T>::callback(const DataTypeSharedPtr msg) {
  if (data_.mul == 0) {
    monitor_exist_lidar(data_.topic);
  }
  timer_.set();
}

template <class T>
void SensorState<T>::monitor_exist_lidar(const std::string &str) {
  std::ofstream fout;
  YAML::Node config = YAML::LoadFile(topic_config_path_);
  fout.open(topic_config_path_, std::ios::out);
  if (!fout.is_open()) {
    std::cout << "can't open " + topic_config_path_ << std::endl;
  }

  fout << "topic_state:" << std::endl;

  for (int i = 0; config["topic_state"].size() > i; i++) {
    std::string value_enable{"true"};
    std::string value_mul{config["topic_state"][i]["mul"].as<std::string>()};
    if (config["topic_state"][i]["enable"].as<bool>() == 0) {
      value_enable = "false";
    }
    if (str == config["topic_state"][i]["topic"].as<std::string>()) {
      value_mul = "1";
    }

    fout << " "
         << " - { topic: \""
         << config["topic_state"][i]["topic"].as<std::string>()
         << "\", type: \"" << config["topic_state"][i]["type"].as<std::string>()
         << "\", name: \"" << config["topic_state"][i]["name"].as<std::string>()
         << "\", timeout: " << config["topic_state"][i]["timeout"].as<double>()
         << ", enable: " << value_enable << ", mul: " << value_mul << "}"
         << std::endl;
  }
  fout.close();
}

//-----------------------------------------
RobotConfig::RobotConfig() : Node("robot_config") { init(); }

RobotConfig::~RobotConfig() = default;

void RobotConfig::init() {
  RCLCPP_INFO_STREAM(this->get_logger(), "robot config init");

  //获取cti_path
  cti_file_path_ = this->declare_parameter<std::string>("CTI_FILE_PATH", "");
  parameter_server_["CTI_FILE_PATH"] = cti_file_path_;
  cti_topic_config_path_ =
      this->declare_parameter<std::string>("CTI_TOPIC_CONFIG_FILE_PATH", "");
  parameter_server_["CTI_TOPIC_CONFIG_FILE_PATH"] = cti_topic_config_path_;
  cti_vehicle_config_file_path_ =
      this->declare_parameter<std::string>("CTI_VEHICLE_CONFIG_FILE_PATH", "");
  parameter_server_["CTI_VEHICLE_CONFIG_FILE_PATH"] =
      cti_vehicle_config_file_path_;

  cti_static_tf_file_path_ = this->declare_parameter<std::string>(
      "CTI_STATIC_TF_CONFIG_FILE_PATH", "");
  parameter_server_["CTI_STATIC_TF_CONFIG_FILE_PATH"] =
      cti_static_tf_file_path_;

  //发布话题
  map_data_pub_ = this->create_publisher<cti_msgs::msg::MapDataConfig>(
      "/robot_config/map_data_config", rclcpp::QoS{1}.transient_local());

  robot_parm_pub_ = this->create_publisher<cti_msgs::msg::RobotParamConfig>(
      "/robot_config/robot_param_config", rclcpp::QoS{1}.transient_local());
  robot_version_pub_ =
      this->create_publisher<cti_msgs::msg::RobotVersionDisplay>(
          "/cti/robot_config/robotVersion", rclcpp::QoS{1}.transient_local());
  sensor_state_pub_ = this->create_publisher<cti_msgs::msg::BoxState>(
      "/cti/robot_config/sensorState", rclcpp::QoS{1});
  pub_errorArray_publisher =
      this->create_publisher<cti_msgs::msg::ErrorStatusArray>(
          "/cti/error_status/get", rclcpp::QoS{1});
  pub_error_msg = this->create_publisher<cti_msgs::msg::ErrorStatus>(
      "/cti/error_status/set", rclcpp::QoS{1}.transient_local());
  parameter_server_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/robot_config/parameter_server", rclcpp::QoS{1}.transient_local());

  //订阅话题
  robot_version_sub_ =
      this->create_subscription<cti_msgs::msg::RobotVersionDisplay>(
          "/cti/fpga_serial/operationControlVersion",
          rclcpp::QoS{1}.transient_local(),
          std::bind(&RobotConfig::robotVersionCallback, this,
                    std::placeholders::_1));
  robot_env_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/cti/robot_env", rclcpp::QoS{1}.transient_local(),
      std::bind(&RobotConfig::robotEnvCallback, this, std::placeholders::_1));
  localizer_state_sub_ =
      this->create_subscription<cti_msgs::msg::RobotLocalizerState>(
          "/cti_localizer_state", rclcpp::QoS{1},
          std::bind(&RobotConfig::localizerStateCallback, this,
                    std::placeholders::_1));
  sub_error_status_ = this->create_subscription<cti_msgs::msg::ErrorStatus>(
      "/cti/error_status/set", rclcpp::QoS{1},
      std::bind(&RobotConfig::error_status_Callback, this,
                std::placeholders::_1));

  parameter_server_sub = this->create_subscription<std_msgs::msg::String>(
      "/robot_config/parameter_server",
      rclcpp::QoS{10}.transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg)
      { parameterServerCallback(msg); });


  // ros2 topic echo --qos-profile services_default --qos-durability
  // transient_local  /cti/error_status/set 定时器
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   [this]() { this->timercallback(); });

  // 发布静态tf
  getStaticTfParams(cti_static_tf_file_path_);
  tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  // makeUltStaticTransforms();

  declareVehicleConfigParams();
  load_env_param();
  pubParameterServer();
  publishMapconfig(run_env_);

  //--读取参数--
  std::vector<TopicState> topicStates;
  getTopicStateParams(topicStates);
  sensor_states_.clear();
  for (auto &topicState : topicStates) {
    if (topicState.type == "sensor_msgs::PointCloud2") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::PointCloud2>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } else if (topicState.type == "sensor_msgs::LaserScan") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::LaserScan>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    } else if (topicState.type == "sensor_msgs::Imu") {
      std::shared_ptr<StateBase> ptr =
          std::make_shared<SensorState<sensor_msgs::msg::Imu>>(
              this, topicState, cti_topic_config_path_);
      sensor_states_.emplace_back(ptr);
    }
  }

  pubErrorMsg("SENSOR", 0, 11, " 没有初始化 ");
}


void RobotConfig::parameterServerCallback(const std_msgs::msg::String::SharedPtr msg){

  std::cout<<"parameterServerCallback"<<std::endl;
  Json::Value parameter_server;
  Json::Reader reader;
  reader.parse(msg->data, parameter_server);
  float vehicleModel_higth = parameter_server["vehicleModel.higth"].asFloat();
  float vehicleModel_shaft_length = parameter_server["vehicleModel.shaft_length"].asFloat();
  float vehicleModel_wheel_radius = parameter_server["vehicleModel.wheel_radius"].asFloat();
  float vehicleModel_rearRigheLidarX = parameter_server["vehicleModel.rearRigheLidarX"].asFloat();
  float vehicleModel_base_to_front = parameter_server["vehicleModel.base_to_front"].asFloat();


  StaticTfState state;
  //base_link to center_pose
  state.frame_id = "base_link";
  state.child_frame_id = "center_pose";
  state.translation_x = vehicleModel_shaft_length/2;
  state.translation_z = vehicleModel_higth/2;
  state.rotation_w = 1;
  staticTfStates_.emplace_back(state);
  //base_link to rear_pose
  state.frame_id = "base_link";
  state.child_frame_id = "rear_pose";
  state.translation_x = vehicleModel_rearRigheLidarX;
  state.translation_z = vehicleModel_wheel_radius;
  state.rotation_w = 1;
  staticTfStates_.emplace_back(state);
  //base_link to front_pose
  state.frame_id = "base_link";
  state.child_frame_id = "front_pose";
  state.translation_x = vehicleModel_base_to_front;
  state.translation_z = vehicleModel_wheel_radius;
  state.rotation_w = 1;
  staticTfStates_.emplace_back(state);

}


void RobotConfig::load_env_param() {
  //读取vmap version版本
  std::ifstream fout;
  // boot_sh/version
  std::string read_ver_path{cti_file_path_ + "/version"};
  fout.open(read_ver_path);
  if (!fout.is_open()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "打开" + read_ver_path + "失败,");
    run_env_ = "sty";
    parameter_server_["CTI_RUN_ENV"] = "sty";
    parameter_server_["CTI_RUN_VER"] = "v3.0";
    parameter_server_["ROBOT_N"] = "CTI-0";
  } else {
    std::vector<std::string> nums;
    std::string tmp;
    while (getline(fout, tmp)) {
      char *str = (char *)tmp.c_str();  // string --> char
      const char *split = ":";
      char *p = strtok(str, split);  //:分隔依次取出

      while (p != NULL) {
        nums.push_back(p);
        p = strtok(NULL, split);
      }
      run_env_ = nums[1];
      parameter_server_["CTI_RUN_ENV"] = nums[1];
      parameter_server_["CTI_RUN_VER"] = nums[0] + "." + nums[2];
      parameter_server_["ROBOT_N"] = nums[3];
    }
  }
  fout.close();

  std::string read_nav_ver_path{cti_file_path_ + "/.cti_version"};
  std::string nav_env_;
  fout.open(read_nav_ver_path);

  if (!fout.is_open()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "打开" + read_ver_path + "失败,");
    nav_env_ = "null";
    parameter_server_["CTI_NAV_VER"] = nav_env_;
  } else {
    getline(fout, nav_env_);
    parameter_server_["CTI_NAV_VER"] = nav_env_;
  }
  fout.close();


  // LOG
  std::string read_log_path{cti_file_path_ + "/ctilog"};
  parameter_server_["CTI_RUN_LOG_PATH"] = read_log_path;
  // cti_map
  std::string read_vmap_path{cti_file_path_ + "/cti_vmap"};
  parameter_server_["CTI_RUN_MAP_PATH"] = read_vmap_path;
  cti_vmap_path_ = read_vmap_path;
  // cti_vmap
  std::string read_map_path{cti_file_path_ + "/cti_map"};
  cti_map_path_ = read_map_path;
  parameter_server_["CTI_MAP_PATH"] = read_map_path;

  // cti_vmap_ver
  std::string read_vmapVer_path{cti_vmap_path_ + "/" + run_env_ +
                                "/version_vmap"};
  fout.open(read_vmapVer_path);
  if (!fout.is_open()) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "打开" + read_vmapVer_path + "失败,");
    robot_version_.map_version = "v1.0";
    parameter_server_["CTI_RUN_MAP_VER"] = "v1.0";
  } else {
    std::string tmp2;
    getline(fout, tmp2);
    robot_version_.map_version = tmp2;
    parameter_server_["CTI_RUN_MAP_VER"] = tmp2;
  }
  fout.close();

  // cti_sw_
  std::string read_swVer_path{cti_file_path_ + "/install/version"};
  fout.open(read_swVer_path);
  if (!fout.is_open()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "打开" + read_swVer_path + "失败,");
    robot_version_.nav_version = "V0.0.0";
    parameter_server_["CTI_SW_VER"] = "V0.0.0";
  } else {
    std::string tmp3;
    getline(fout, tmp3);
    robot_version_.nav_version = tmp3;
    parameter_server_["CTI_SW_VER"] = tmp3;
  }
  fout.close();

  // cameralow_flag
  std::string read_cameralow_flag{
      cti_vehicle_config_file_path_ +
      "/candela_CameraLow_to_LidarLow_extrinsic.yml"};
  fout.open(read_cameralow_flag);
  if (!fout.is_open()) {
    parameter_server_["CTI_CAMERALOW_FLAG"] = 0;
  } else {
    parameter_server_["CTI_CAMERALOW_FLAG"] = 1;
  }
  fout.close();
}

void RobotConfig::pubErrorMsg(const std::string &modular_name,
                              const uint &error_code, const uint &error_level,
                              const std::string &error_msg) {
  cti_msgs::msg::ErrorStatus error_msgs;
  error_msgs.stamp = this->now();
  error_msgs.module_name = modular_name;
  error_msgs.error_code = error_code;
  error_msgs.error_info = error_msg;
  error_msgs.level = error_level;
  pub_error_msg->publish(error_msgs);
}

void RobotConfig::getTopicStateParams(std::vector<TopicState> &topicStates) {
  return;
  try {
    auto yaml_node = YAML::LoadFile(cti_topic_config_path_);
    auto configs = yaml_node["topic_state"];
    topicStates.clear();
    for (const auto &config : configs) {
      TopicState state;
      state.topic = config["topic"].as<std::string>();
      state.name = config["name"].as<std::string>();
      state.type = config["type"].as<std::string>();
      state.timeout = config["timeout"].as<double>();
      state.enable = config["enable"].as<bool>();
      state.mul = config["mul"].as<int>();
      topicStates.emplace_back(state);
    }

  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
  }
}

void RobotConfig::getStaticTfParams(const std::string &path) {
  try {
    auto yaml_node = YAML::LoadFile(path);
    auto configs = yaml_node["static_tf"];
    for (const auto &config : configs) {
      StaticTfState state;
      state.frame_id = config["frame_id"].as<std::string>();
      state.child_frame_id = config["child_frame_id"].as<std::string>();
      state.translation_x = config["translation_x"].as<double>();
      state.translation_y = config["translation_y"].as<double>();
      state.translation_z = config["translation_z"].as<double>();
      state.rotation_x = config["rotation_x"].as<double>();
      state.rotation_y = config["rotation_y"].as<double>();
      state.rotation_z = config["rotation_z"].as<double>();
      state.rotation_w = config["rotation_w"].as<double>();
      staticTfStates_.emplace_back(state);
    }

  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
  }
}

void RobotConfig::declareVehicleConfigParams() {
  try {
    YAML::Node vehicle_ = YAML::LoadFile(cti_vehicle_config_file_path_);
    YAML::Node vehicle_info = vehicle_["/**"]["ros__parameters"];
    for (auto const &param_namespace : vehicle_info) {
      std::string name_space = param_namespace.first.as<std::string>();
      for (auto const &param : param_namespace.second) {
        std::string param_name = param.first.as<std::string>();
        std::string declare_param_name = name_space + "." + param_name;
        parameter_server_[declare_param_name] = param.second.as<double>();
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Declare Vehicle Parameter: "
                               << declare_param_name << "---"
                               << parameter_server_[declare_param_name]);
      }
    }

  } catch (const std::exception &ex) {
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
  }
}

void RobotConfig::robotEnvCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data == "") {
    RCLCPP_INFO_STREAM(get_logger(), "接受到环境地图为空");
  }
  run_env_ = msg->data;
  parameter_server_["CTI_RUN_ENV"] = run_env_;
  std::ifstream fout;
  std::string read_vmapVer_path{cti_vmap_path_ + "/" + run_env_ +
                                "/version_vmap"};
  fout.open(read_vmapVer_path);
  if (!fout.is_open()) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "打开" + read_vmapVer_path + "失败,");
    parameter_server_["CTI_RUN_MAP_VER"] = "v1.0";
    robot_version_.map_version = "v1.0";
  } else {
    std::string tmp2;
    getline(fout, tmp2);
    parameter_server_["CTI_RUN_MAP_VER"] = tmp2;
    robot_version_.map_version = tmp2;
  }
  fout.close();
  RCLCPP_INFO_STREAM(get_logger(), "发布环境地图:" << run_env_);
  publishMapconfig(run_env_);
  pubParameterServer();
}

void RobotConfig::robotVersionCallback(
    const cti_msgs::msg::RobotVersionDisplay::SharedPtr msg) {
  robot_version_.operation_control_version = msg->operation_control_version;
  robot_version_pub_->publish(robot_version_);
}

void RobotConfig::localizerStateCallback(
    const cti_msgs::msg::RobotLocalizerState::SharedPtr msg) {
  localizer_state_ = msg->id;
}

//发布函数
void RobotConfig::publishMapconfig(const std::string &env) {
  cti_msgs::msg::MapDataConfig config;
  config.cross_walk = cti_vmap_path_ + "/" + env + "/" + "crosswalk.csv";
  config.lane = cti_vmap_path_ + "/" + env + "/" + "lane.csv";
  config.signal = cti_vmap_path_ + "/" + env + "/" + "signal.csv";
  config.road_edge = cti_vmap_path_ + "/" + env + "/" + "roadedge.csv";
  config.path_goal = cti_vmap_path_ + "/" + env + "/" + "pathgoal.csv";
  config.stop_point = cti_vmap_path_ + "/" + env + "/" + "stoppoint.csv";
  config.wait_line = cti_vmap_path_ + "/" + env + "/" + "waitline.csv";
  config.junction = cti_vmap_path_ + "/" + env + "/" + "junction.csv";
  config.gid = cti_vmap_path_ + "/" + env + "/" + "gid.csv";
  config.dece_zone = cti_vmap_path_ + "/" + env + "/" + "decezone.csv";
  config.gate = cti_vmap_path_ + "/" + env + "/" + "gate.csv";
  config.clear_area = cti_vmap_path_ + "/" + env + "/" + "cleararea.csv";
  config.spray_area = cti_vmap_path_ + "/" + env + "/" + "sprayarea.csv";
  config.elevator = cti_vmap_path_ + "/" + env + "/" + "elevator.csv";
  config.converge_point =
      cti_vmap_path_ + "/" + env + "/" + "convergepoint.csv";
  config.lanelet = cti_vmap_path_ + "/" + env + "/" + "hdmap.osm";
  config.pcd = cti_map_path_ + "/" + env + "/" + env + ".pcd";
  config.ndt2gps = cti_map_path_ + "/" + env + "/" + "calibration.yaml";
  config.attribute_area =
      cti_vmap_path_ + "/" + env + "/" + "attributearea.csv";
  map_data_pub_->publish(config);
}

//发布所有传感器状态信息
void RobotConfig::publishSensorState() {
  cti_msgs::msg::BoxState sensor_states;
  sensor_states.header.stamp = this->now();

  int numb_sensor_data = 0;

  for (const auto &state : sensor_states_) {
    if (state->getMul() == 0) {
      continue;
    }

    cti_msgs::msg::TabState tabstate;
    tabstate.id = numb_sensor_data;
    tabstate.status = state->getState();
    tabstate.name = state->getName();

    if (tabstate.status == StateType::ERROR) {
      tabstate.message = " no init";
      // RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(
      //     get_logger(), *(get_clock()), 5000,
      //     "lidar.name:" << tabstate.name << "  lidar.statu:no init");
      RCLCPP_INFO_STREAM(
          get_logger(),
          "lidar.name:" << tabstate.name << "  lidar.statu:no init");
      pubErrorMsg("SENSOR", 0, 11, tabstate.name + " 没有初始化 ");
    } else if (tabstate.status == StateType::OVERTIME) {
      tabstate.message = " over time";
      // RCLCPP_INFO_STREAM_SKIPFIRST_THROTTLE(
      //     get_logger(), *(get_clock()), 5000,
      //     "lidar.name:" << tabstate.name << "  lidar.statu:over time");
      RCLCPP_INFO_STREAM(
          get_logger(),
          "lidar.name:" << tabstate.name << "  lidar.statu:over time");
      pubErrorMsg("SENSOR", 1, 11, tabstate.name + " 超时 ");
    } else {
      tabstate.message = " is normal";
    }
    numb_sensor_data++;
    sensor_states.states.push_back(tabstate);
  }
  sensor_state_pub_->publish(sensor_states);
}

void RobotConfig::error_status_Callback(
    const cti_msgs::msg::ErrorStatus::SharedPtr msg) {
  static double pub_time{this->now().seconds()};

  if (error_msg_array.data.size() == 0)  //初始化
  {
    pub_time = this->now().seconds();
    error_msg_array.data.push_back(*msg);
  }

  if (this->now().seconds() - pub_time >
      0.9)  //超过  ，还没收到信息，一般是已经恢复正常了
  {
    pub_errorArray_publisher->publish(error_msg_array);
    error_msg_array.data.clear();
    pub_errorArray_publisher->publish(error_msg_array);
    release_error_msg = false;
    return;
  }

  bool flag{false};
  for (auto sub : error_msg_array.data)  //重复的信息过滤
  {
    if (sub.module_name == msg->module_name &&
        sub.error_code == msg->error_code &&
        sub.error_info == msg->error_info) {
      flag = true;
      break;
    }
  }

  if (!flag) {
    if (!msg->module_name.empty()) {
      error_msg_array.data.push_back(*msg);
    }
  }

  if (this->now().seconds() - pub_time > 0.5)  // 2hz 发送一次收集的错误码
  {
    pub_errorArray_publisher->publish(error_msg_array);
    error_msg_array.data.clear();
    release_error_msg = false;
    return;
  }

  release_error_msg = true;
  time_monitor_err = this->now().seconds();
}

void RobotConfig::monitor_release_data() {
  if (release_error_msg == 1) {
    if (this->now().seconds() - time_monitor_err > 1) {
      cti_msgs::msg::ErrorStatus::SharedPtr msg;
      error_status_Callback(msg);
    }
  }
}

void RobotConfig::pubParameterServer() {
  Json::FastWriter writer;
  std_msgs::msg::String msg;
  msg.data = writer.write(parameter_server_);
  parameter_server_pub_->publish(msg);
}

void RobotConfig::makeUltStaticTransforms() {
  rclcpp::Time now = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped t;

  for (auto state : staticTfStates_) {
    t.header.stamp = now;
    t.header.frame_id = state.frame_id;
    t.child_frame_id = state.child_frame_id;

    t.transform.translation.x = state.translation_x;
    t.transform.translation.y = state.translation_y;
    t.transform.translation.z = state.translation_z;

    t.transform.rotation.x = state.rotation_x;
    t.transform.rotation.y = state.rotation_y;
    t.transform.rotation.z = state.rotation_z;
    t.transform.rotation.w = state.rotation_w;

    tf_publisher_->sendTransform(t);
  }
}

//-------------- timer ---------------------------------------
void RobotConfig::timercallback() {
  publishSensorState();
  monitor_release_data();
  makeUltStaticTransforms();
}