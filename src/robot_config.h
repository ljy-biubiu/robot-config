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
#include "string.h"
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <jsoncpp/json/json.h>
#include <std_msgs/String.h>
#include "cti_msgs/RobotParamConfig.h"
#include "cti_msgs/MapDataConfig.h"
#include "cti_msgs/SceneChange.h"
#include "cti_msgs/VehicleModel.h"
#include "cti_msgs/RobotVersionDisplay.h"
#include "cti_msgs/AllSensorStatus.h"
#include "cti_msgs/SensorState.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "yaml-cpp/yaml.h"
#include "pwd.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <vector>
#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/Imu.h"
#include "cti_msgs/RobotLocalizerState.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"

#include "cti_msgs/BoxState.h"
#include "cti_msgs/TabState.h"
#include "cti_msgs/Data.h"
#include "cti_msgs/DataArray.h"
#include "cti_msgs/ErrorStatus.h"
#include "cti_msgs/ErrorStatusArray.h"
#include "std_msgs/Bool.h"

#include "ctilog/log.hpp"
#include "ctilog/loghelper.cpp.hpp"

// #include "uint8.h"

#include "core.h"

using namespace std;
using namespace cti::log;

template <typename T>
T getParam(const YAML::Node &node, const string &name, const T &defaultValue)
{
    T v;
    try
    {
        v = node[name].as<T>();
    }
    catch (std::exception e)
    {
        v = defaultValue;
    }
    return v;
}

struct TopicState
{
    std::string name{""};
    std::string topic{""};
    std::string type{""};
    float timeout{0};
    bool enable{false};
    int mul{0};
};

class State
{
public:
    enum
    {
        ERROR = -1,   //错误，开机未读取到数据
        NORMAL = 0,   //正常
        OVERTIME = 1, //超时
    };
    struct StateTimer
    {
        void set()
        {
            state = true;
            start_timer = ros::Time::now().toSec();
        }
        double getDuration()
        {
            return ros::Time::now().toSec() - start_timer;
        }
        bool state{false};
        double start_timer{0};
    };
    virtual void init(ros::NodeHandle &nh_) = 0;
    virtual int state() = 0;
    virtual std::string getTopic() = 0;
    virtual std::string getName() = 0;
    virtual std::string getType() = 0;
    virtual float getTimeout() = 0;
    virtual bool getEnable() = 0;
    virtual int getMul() = 0;
    virtual void monitor_exist_lidar(const string &str) = 0;
    StateTimer timer;
};

template <class T>
class SensorState : public State
{
public:
    SensorState(ros::NodeHandle &nh_, TopicState &state,std::string topic_state_yaml_path) : data(state),topic_state_yaml_path_(topic_state_yaml_path)
    {
        init(nh_);
    }

private:
    void init(ros::NodeHandle &nh_);
    int state();
    std::string getTopic();
    std::string getName();
    std::string getType();
    float getTimeout();
    bool getEnable();
    int getMul();
    void monitor_exist_lidar(const string &str);
    void callback(const T &msg);

private:
    std::string topic_state_yaml_path_;
    ros::Subscriber sub;
    TopicState data;
};

class RobotConfig
{
public:
    RobotConfig();
    ~RobotConfig();
    void init();

private:
    void publishSensorState();
    void getParams(std::vector<TopicState> &topicStates);
    void assignment(std::string file);
    void log_init(string log_path);
    void publish_map_config(std::string env);
    int if_mul_normal(int);
    //回调函数声明
    void timercallback(const ros::TimerEvent &event);
    void callback_opera_version(const cti_msgs::RobotVersionDisplay::ConstPtr &msg);
    void callback_ndt_pose(const geometry_msgs::PoseStamped &ndt_pose);
    void callback_controlBoard_version_status(const std_msgs::Int8 &controlBoardVersionStatusMsg);
    void callback_localizer_state(const cti_msgs::RobotLocalizerState &localizer_state);
    void callback_robot_env(const std_msgs::String &env);
    void callback_error_status(const cti_msgs::ErrorStatus &msg);
    void pubErrorMsg(const std::string &modular_name, const uint &error_code, const uint &error_level, const std::string &error_msg);
    void monitor_release_data();
    void check_mpaAndvmap_version();
    void callback_error_msg1();
    void callback_error_msg2();
    void callback_error_msg3();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    //订阅
    ros::Subscriber sub_error_status_;
    ros::Subscriber sub_opera_version_;
    ros::Subscriber sub_ndt_pose_;
    ros::Subscriber sub_localizer_state_;
    ros::Subscriber sub_robot_env_;
    //发布
    ros::Publisher pub_map_data_;
    ros::Publisher pub_map_config_;
    ros::Publisher pub_robot_param_;
    ros::Publisher pub_robot_version_;
    ros::Publisher pub_base_pose_;
    ros::Publisher pub_sensor_state_;
    ros::Publisher pub_error_msg;
    ros::Publisher pub_mapAvmap_ver_err;
    ros::Publisher pub_lidar_states;
    ros::Publisher error_publisher;

    ros::Publisher pub_error1_msg;
    ros::Publisher pub_error2_msg;
    ros::Publisher pub_error3_msg;

    //定时器
    ros::Timer timer;
    //系统参数
    cti_msgs::RobotVersionDisplay robotVersion_;
    tf::TransformListener tf_listen_; //监听者
    geometry_msgs::PoseStamped base_pose_;
    geometry_msgs::PoseStamped ndt_pose_;
    cti_msgs::ErrorStatusArray error_msg_array;

    std::string cti_map_path_;
    std::string cti_vmap_path_;
    std::string robot_config_file_;
    std::string run_env_;
    std::vector<State *> sensorStates_;

    int localizer_state_;
    bool release_error_msg{false};
    double time_monitor_err{0};
};
#endif
