/*************************************************************************
	> File Name: robot_config.cpp
	> Author: ma6174
	> Mail: ma6174@163.com 
	> Created Time: 2018年08月21日 星期二 10时58分51秒
 ************************************************************************/

#include <iostream>
#include "robot_config.h"
#include "std_msgs/String.h"
#include <jsoncpp/json/json.h>

using namespace std;
using namespace cti::log;
constexpr char const *kN = "config";
const std::string home_str = std::string(std::getenv("HOME"));

//--------------------------------------
template <class T>
void SensorState<T>::init(ros::NodeHandle &nh_)
{
    if (!data.topic.empty())
    {
        sub = nh_.subscribe(data.topic, 1, &SensorState::callback, this);
    }
}

template <class T>
int SensorState<T>::state()
{
    int stat = State::ERROR;
    if (data.enable)
    {
        if (timer.state)
        {
            if (timer.getDuration() > data.timeout)
            {
                stat = State::OVERTIME;
            }
            else
            {
                stat = State::NORMAL;
            }
        }
    }
    else
    {
        stat = State::NORMAL;
    }
    return stat;
}
template <class T>
std::string SensorState<T>::getTopic()
{
    return data.topic;
}

template <class T>
std::string SensorState<T>::getName()
{
    return data.name;
}

template <class T>
std::string SensorState<T>::getType()
{
    return data.type;
}

template <class T>
float SensorState<T>::getTimeout()
{
    return data.timeout;
}

template <class T>
bool SensorState<T>::getEnable()
{
    return data.enable;
}

template <class T>
int SensorState<T>::getMul()
{
    return data.mul;
}

template <class T>
void SensorState<T>::monitor_exist_lidar(const string &str)
{
    ofstream fout;// home_str + "/log/robot_config.log";
    YAML::Node config = YAML::LoadFile(topic_state_yaml_path_);
    fout.open(topic_state_yaml_path_, ios::out);

    if (!fout.is_open())
    {
        // config = YAML::LoadFile(topic_state_yaml_path_);
        // fout.open(topic_state_yaml_path_);
        Info("修改 topic_state_yaml 文件失败");
        return;
    }

    fout << "topic_state:" << std::endl;

    for (int i = 0; config["topic_state"].size() > i; i++)
    {
        std::string value_enable{"true"};
        std::string value_mul{config["topic_state"][i]["mul"].as<std::string>()};
        if (config["topic_state"][i]["enable"].as<bool>() == 0)
        {
            value_enable = "false";
        }
        if (str == config["topic_state"][i]["topic"].as<std::string>())
        {
            value_mul = "1";
        }

        fout << " "
             << " - { topic: \"" << config["topic_state"][i]["topic"].as<std::string>()
             << "\", type: \"" << config["topic_state"][i]["type"].as<std::string>()
             << "\", name: \"" << config["topic_state"][i]["name"].as<std::string>()
             << "\", timeout: " << config["topic_state"][i]["timeout"].as<double>()
             << ", enable: " << value_enable << ", mul: " << value_mul << "}" << std::endl;
    }

    fout.close();
}

template <class T>
void SensorState<T>::callback(const T &msg)
{
    if (data.mul == 0)
    {
        monitor_exist_lidar(data.topic);
    }
    timer.set();
}

//-----------------------------------------
RobotConfig::RobotConfig() : pnh_("~")
{
}

RobotConfig::~RobotConfig()
{
}

void RobotConfig::init()
{
    std::string log_path_;
    if (nh_.getParam("CTI_RUN_LOG_PATH", log_path_))
    {
        log_path_ += "/robot_config.log";
    }
    else
    {
        log_path_ = home_str + "/log/robot_config.log";
    }
    std::string topic_state_yaml_path_;
    if (!pnh_.getParam("topic_state_yaml_path", topic_state_yaml_path_))
    {
        topic_state_yaml_path_ = "/opt/cti/kinetic/share/robot_config/config/topic_state.yaml";
    }

    log_init(log_path_); //初始化系统日志
    Info(" *****robot config start*****");
    //--读取参数--
    std::vector<TopicState> topicStates;
    getParams(topicStates);
    State *sensorState = NULL;
    sensorStates_.clear();
    for (int i = 0; i < topicStates.size(); i++)
    {
        sensorState = NULL;
        if (topicStates[i].type == "sensor_msgs::PointCloud2")
        {
            sensorState = new SensorState<sensor_msgs::PointCloud2>(nh_, topicStates[i],topic_state_yaml_path_);
        }
        else if (topicStates[i].type == "sensor_msgs::LaserScan")
        {
            sensorState = new SensorState<sensor_msgs::LaserScan>(nh_, topicStates[i],topic_state_yaml_path_);
        }
        else if (topicStates[i].type == "sensor_msgs::Imu")
        {
            sensorState = new SensorState<sensor_msgs::Imu>(nh_, topicStates[i],topic_state_yaml_path_);
        }
        if (sensorState)
        {
            sensorStates_.push_back(sensorState);
        }
    }
    //发布话题
    pub_map_data_ = nh_.advertise<cti_msgs::MapDataConfig>("/robot_config/map_data_config", 1, true);
    pub_map_config_ = nh_.advertise<cti_msgs::DataArray>("/robot_config/map_config", 1, true);
    pub_robot_param_ = nh_.advertise<cti_msgs::RobotParamConfig>("/robot_config/robot_param_config", 1, true);
    pub_robot_version_ = nh_.advertise<cti_msgs::RobotVersionDisplay>("/cti/robot_config/robotVersion", 1, true);
    pub_base_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/cti/robot_config/base_link_pose", 1, true);
    pub_sensor_state_ = nh_.advertise<cti_msgs::BoxState>("/cti/robot_config/sensorState", 1, true);
    pub_error_msg = nh_.advertise<cti_msgs::ErrorStatusArray>("/cti/error_status/get", 5, true);
    pub_mapAvmap_ver_err = nh_.advertise<std_msgs::Bool>("/cti/robot_config/mapAvmap_ver_Err", 1, true);
    error_publisher = nh_.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 20, true);

    pub_error1_msg = nh_.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 3, true);
    pub_error2_msg = nh_.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 3, true);
    pub_error3_msg = nh_.advertise<cti_msgs::ErrorStatus>("/cti/error_status/set", 3, true);

    //订阅话题cti_msgs/ErrorStatus
    sub_error_status_ = nh_.subscribe("/cti/error_status/set", 10, &RobotConfig::callback_error_status, this);
    sub_opera_version_ = nh_.subscribe("/cti/fpga_serial/operationControlVersion", 1, &RobotConfig::callback_opera_version, this);
    sub_robot_env_ = nh_.subscribe("/cti/robot_env", 1, &RobotConfig::callback_robot_env, this);
    //sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 1, &RobotConfig::callback_ndt_pose, this);
    sub_localizer_state_ = nh_.subscribe("/cti_localizer_state", 1, &RobotConfig::callback_localizer_state, this);
    //定时器
    timer = nh_.createTimer(ros::Duration(0.1), &RobotConfig::timercallback, this);
    //--sensorStates_
    assignment(robot_config_file_);
    // pubErrorMsg("SENSOR", 1, 11, " 超时 ");
}

void RobotConfig::getParams(std::vector<TopicState> &topicStates)
{
    if (!nh_.getParam("CTI_RUN_MAP_PATH", cti_vmap_path_))
    {
        cti_vmap_path_ = home_str + "/cti_vmap";
    }
    if (!nh_.getParam("CTI_MAP_PATH", cti_map_path_))
    {
        cti_map_path_ = home_str + "/cti_map";
    }
    if (!nh_.getParam("CTI_SW_VER", robotVersion_.navVersion))
    {
        robotVersion_.navVersion = "";
    }
    if (!nh_.getParam("CTI_RUN_ENV", run_env_))
    {
        run_env_ = "wkyc";
    }
    if (!nh_.getParam("CTI_RUN_MAP_VER", robotVersion_.mapVersion))
    {
        robotVersion_.mapVersion = "";
    }
    pnh_.param<std::string>("robot_config_file", robot_config_file_, "");
    //--
    std::string sub_name = "topic_state";
    XmlRpc::XmlRpcValue sub_config;
    if (!pnh_.getParam(sub_name, sub_config))
    {
        ROS_WARN("Could not load the configuration from parameter %s,are you sure it", sub_name.c_str());
        return;
    }
    if (sub_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("XmlRpcType:%d", sub_config.getType());
        return;
    }

    topicStates.clear();
    for (int k = 0; k < sub_config.size(); k++)
    {
        TopicState state;
        if (sub_config[k].hasMember("topic") && sub_config[k]["topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            state.topic = static_cast<std::string>(sub_config[k]["topic"]);
            ROS_INFO("topic:%s", state.topic.c_str());
        }
        if (sub_config[k].hasMember("name") && sub_config[k]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            state.name = static_cast<std::string>(sub_config[k]["name"]);
            ROS_INFO("name:%s", state.name.c_str());
        }
        if (sub_config[k].hasMember("type") && sub_config[k]["type"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            state.type = static_cast<std::string>(sub_config[k]["type"]);
            ROS_INFO("type:%s", state.type.c_str());
        }
        if (sub_config[k].hasMember("timeout") && sub_config[k]["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            state.timeout = static_cast<double>(sub_config[k]["timeout"]);
            ROS_INFO("timeout:%f", state.timeout); 
            Info("timeout:" << state.timeout);
        }
        if (sub_config[k].hasMember("enable") && sub_config[k]["enable"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            state.enable = static_cast<bool>(sub_config[k]["enable"]);
            ROS_INFO("enable:%s", state.enable ? "true" : "false");
        }
        if (sub_config[k].hasMember("mul") && sub_config[k]["mul"].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            state.mul = static_cast<int>(sub_config[k]["mul"]);
            ROS_INFO("mul:%s", state.enable ? "true" : "false");
        }
        topicStates.push_back(state);
        ROS_INFO("----------------");
    }
    printf("size:%ld\n", topicStates.size());
}

void RobotConfig::assignment(std::string file)
{
    if (file.empty())
    {
        return;
    }
    cti_msgs::RobotParamConfig robot_config_;
    std::unique_ptr<std::istream> is;
    is.reset(new std::ifstream(file.c_str(), std::ifstream::in));
    YAML::Node robot_config = YAML::Load(*is);

    YAML::Node base_info = robot_config["base_info"];
    YAML::Node type_info = robot_config["type_config"];
    YAML::Node scene_info = robot_config["scene_config"];
    YAML::Node sensor_info = robot_config["sensor_config"];
    int robot_type = getParam<int>(base_info, "robot_type", 0);
    int scene_type = getParam<int>(base_info, "scene", 0);

    // --------------------------------------------------------
    robot_config_.robot_type = getParam<int>(base_info, "robot_type", 0);
    robot_config_.scene = getParam<int>(base_info, "scene", 0);
    robot_config_.max_vx = getParam<float>(type_info, "max_vx", 0);
    robot_config_.max_vz = getParam<float>(type_info, "max_vz", 0);
    robot_config_.feedback_flag = getParam<int>(type_info, "feedback_flag", 0);
    robot_config_.acc_x_up = getParam<float>(type_info, "acc_x_up", 0);
    robot_config_.acc_x_down = getParam<float>(type_info, "acc_x_down", 0);
    robot_config_.acc_z = getParam<float>(type_info, "acc_z", 0);
    robot_config_.acc_for_curva = getParam<float>(type_info, "acc_for_curva", 0);
    robot_config_.acc_for_x = getParam<float>(type_info, "acc_for_x", 0);
    robot_config_.dis_for_back_global = getParam<float>(type_info, "dis_for_back_global", 0);
    robot_config_.K_stop_enable = getParam<int>(type_info, "K_stop_enable", 0);
    robot_config_.dis_tolerate_for_end = getParam<float>(type_info, "dis_tolerate_for_end", 0);
    robot_config_.dis_tolerate_for_course = getParam<float>(type_info, "dis_tolerate_for_course", 0);
    robot_config_.rad_tolerate = getParam<float>(type_info, "rad_tolerate", 0);
    robot_config_.cos_proportion = getParam<float>(type_info, "cos_/cti/error_status/setproportion", 0);
    robot_config_.rad_to_vz_proportion = getParam<float>(type_info, "rad_to_vz_proportion", 0);
    robot_config_.reverse_vx = getParam<float>(type_info, "reverse_vx", 0);
    robot_config_.guiderail2bumper = getParam<float>(type_info, "guiderail2bumper", 2.0);
    robot_config_.disLoadToUnload = getParam<float>(type_info, "disLoadToUnload", 2.0);
    robot_config_.disToLoad = getParam<float>(type_info, "disToLoad", 0.07);
    pub_robot_param_.publish(robot_config_);
}

void RobotConfig::pubErrorMsg(const std::string &modular_name, const uint &error_code, const uint &error_level, const std::string &error_msg)
{
    cti_msgs::ErrorStatus error_msgs;
    error_msgs.stamp = ros::Time::now();
    error_msgs.module_name = modular_name;
    error_msgs.error_code = error_code;
    error_msgs.error_info = error_msg;
    error_msgs.level = error_level;
    error_publisher.publish(error_msgs);
}

void RobotConfig::callback_robot_env(const std_msgs::String &env)
{
    run_env_ = env.data;
    Info("发布环境地图:" << run_env_);
    publish_map_config(run_env_);
}

void RobotConfig::callback_opera_version(const cti_msgs::RobotVersionDisplay::ConstPtr &msg)
{
    robotVersion_.operationControlVersion = msg->operationControlVersion.c_str();
    pub_robot_version_.publish(robotVersion_);
}

void RobotConfig::callback_localizer_state(const cti_msgs::RobotLocalizerState &localizer_state)
{
    localizer_state_ = localizer_state.id;
}

//发布函数
void RobotConfig::publish_map_config(std::string env)
{
    // cti_msgs::MapDataConfig config;
    // config.crosswalk = cti_vmap_path_ + "/" + env + "/" + "crosswalk.csv";
    // config.lane = cti_vmap_path_ + "/" + env + "/" + "lane.csv";
    // config.signal = cti_vmap_path_ + "/" + env + "/" + "signal.csv";
    // config.roadedge = cti_vmap_path_ + "/" + env + "/" + "roadedge.csv";
    // config.pathgoal = cti_vmap_path_ + "/" + env + "/" + "pathgoal.csv";
    // config.stoppoint = cti_vmap_path_ + "/" + env + "/" + "stoppoint.csv";
    // config.waitline = cti_vmap_path_ + "/" + env + "/" + "waitline.csv";
    // config.junction = cti_vmap_path_ + "/" + env + "/" + "junction.csv";
    // config.gid = cti_vmap_path_ + "/" + env + "/" + "gid.csv";
    // config.decezone = cti_vmap_path_ + "/" + env + "/" + "decezone.csv";
    // config.gate = cti_vmap_path_ + "/" + env + "/" + "gate.csv";
    // config.cleararea = cti_vmap_path_ + "/" + env + "/" + "cleararea.csv";
    // config.sprayarea = cti_vmap_path_ + "/" + env + "/" + "sprayarea.csv";
    // config.elevator = cti_vmap_path_ + "/" + env + "/" + "elevator.csv";
    // config.convergepoint = cti_vmap_path_ + "/" + env + "/" + "convergepoint.csv";
    // config.lanelet = cti_vmap_path_ + "/" + env + "/" + "hdmap.osm";
    // config.pcd = cti_map_path_ + "/" + env + "/" + env + ".pcd";
    // config.ndt2gps = cti_map_path_ + "/" + env + "/" + "calibration.yaml";
    // config.attributearea = cti_map_path_ + "/" + env + "/" + "attributearea.csv";
    // pub_map_data_.publish(config);
    //----
    auto dataConfig = [&](std::string name, std::string value, int type) -> cti_msgs::Data
    {
        cti_msgs::Data data;
        data.name = name;
        data.data = value;
        data.type = type;
        return data;
    };


    cti_msgs::DataArray datas;
    datas.datas.push_back(dataConfig("crosswalk", cti_vmap_path_ + "/" + env + "/" + "crosswalk.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("lane", cti_vmap_path_ + "/" + env + "/" + "lane.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("signal", cti_vmap_path_ + "/" + env + "/" + "signal.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("roadedge", cti_vmap_path_ + "/" + env + "/" + "roadedge.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("pathgoal", cti_vmap_path_ + "/" + env + "/" + "pathgoal.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("stoppoint", cti_vmap_path_ + "/" + env + "/" + "stoppoint.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("waitline", cti_vmap_path_ + "/" + env + "/" + "waitline.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("junction", cti_vmap_path_ + "/" + env + "/" + "junction.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("gid", cti_vmap_path_ + "/" + env + "/" + "gid.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("decezone", cti_vmap_path_ + "/" + env + "/" + "decezone.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("gate", cti_vmap_path_ + "/" + env + "/" + "gate.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("cleararea", cti_vmap_path_ + "/" + env + "/" + "cleararea.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("sprayarea", cti_vmap_path_ + "/" + env + "/" + "sprayarea.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("elevator", cti_vmap_path_ + "/" + env + "/" + "elevator.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("convergepoint", cti_vmap_path_ + "/" + env + "/" + "convergepoint.csv", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("lanelet", cti_vmap_path_ + "/" + env + "/" + "hdmap.osm", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("pcd", cti_map_path_ + "/" + env + "/" + env + ".pcd", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("ndt2gps", cti_map_path_ + "/" + env + "/" + "calibration.yaml", cti_msgs::Data::TYPE_STRING));
    datas.datas.push_back(dataConfig("attributearea", cti_vmap_path_ + "/" + env + "/" + "attributearea.csv", cti_msgs::Data::TYPE_STRING));
    pub_map_config_.publish(datas);
}

//发布所有传感器状态信息
void RobotConfig::publishSensorState()
{
    cti_msgs::BoxState sensor_states;
    int numb_sensor_data = 0;

    sensor_states.header.stamp = ros::Time::now();
    for (auto state : sensorStates_)
    {
        if (state->getMul() == 0)
        {
            continue;
        }

        cti_msgs::TabState tabstate;
        tabstate.id = numb_sensor_data;
        tabstate.status = state->state();
        tabstate.name = state->getName();

        if (tabstate.status == State::ERROR)
        {
            tabstate.message = " no init";
            Info("lidar.name:" << tabstate.name << "  lidar.statu:no init ");
            pubErrorMsg("SENSOR", 0, 11, tabstate.name + " 没有初始化 ");
        }
        else if (tabstate.status == State::OVERTIME)
        {
            tabstate.message = " over time";
            Info("lidar.name:" << tabstate.name << "  lidar.statu:over time ");
            pubErrorMsg("SENSOR", 1, 11, tabstate.name + " 超时 ");
        }
        else
        {
            tabstate.message = " is normal";
        }

        numb_sensor_data++;
        sensor_states.states.push_back(tabstate);
    }
    pub_sensor_state_.publish(sensor_states);
}

void RobotConfig::log_init(std::string log_path)
{
    Logger::setDefaultLogger(log_path);
    Logger::getLogger().setOutputs(Logger::Output::Both);
    Logger::getLogger().setLogLevel(LogLevel::Debu);
    Logger::getLogger().enableTid(false);
    Logger::getLogger().enableIdx(true);
}

void RobotConfig::callback_error_status(const cti_msgs::ErrorStatus &msg)
{
    static double pub_time{ros::Time::now().toSec()};

    if (error_msg_array.data.size() == 0) //初始化
    {
        pub_time = ros::Time::now().toSec();
        error_msg_array.data.push_back(msg);
    }

    if (ros::Time::now().toSec() - pub_time > 0.9) //超过  ，还没收到信息，一般是已经恢复正常了
    {
        std::cout << error_msg_array.data[0].module_name << std::endl;
        pub_error_msg.publish(error_msg_array);
        error_msg_array.data.clear();
        pub_error_msg.publish(error_msg_array);
        release_error_msg = false;
        return;
    }

    bool flag{false};
    for (auto sub : error_msg_array.data) //重复的信息过滤
    {
        if (sub.module_name == msg.module_name && sub.error_code == msg.error_code && sub.error_info == msg.error_info)
        {
            flag = true;
            break;
        }
    }

    if (!flag)
    {
        if (!msg.module_name.empty())
        {
            error_msg_array.data.push_back(msg);
        }
    }

    if (ros::Time::now().toSec() - pub_time > 0.5) //2hz 发送一次收集的错误码
    {
        pub_error_msg.publish(error_msg_array);
        error_msg_array.data.clear();
        release_error_msg = false;
        return;
    }

    release_error_msg = true;
    time_monitor_err = ros::Time::now().toSec();
}

void RobotConfig::monitor_release_data()
{
    if (release_error_msg == 1)
    {
        if (ros::Time::now().toSec() - time_monitor_err > 1)
        {
            cti_msgs::ErrorStatus msg;
            callback_error_status(msg);
        }
    }
}

//-------------- timer ----------  -------------   ----------------
void RobotConfig::timercallback(const ros::TimerEvent &event)
{
    publishSensorState();
    // std::cout<<"----------------------"<<std::endl;
    monitor_release_data();
    // callback_error_msg1();
    // callback_error_msg2();
    // callback_error_msg3();
}

void RobotConfig::callback_error_msg1()
{
    static int i = 1;
    cti_msgs::ErrorStatus error;
    error.error_code = 100;
    error.error_info = "error_1";
    error.module_name = "error_1";
    error.level = 5;
    pub_error1_msg.publish(error);
}

void RobotConfig::callback_error_msg2()
{
    static int q = 100;
    if (q == 0)
    {
        return;
    }
    q--;

    cti_msgs::ErrorStatus error;
    error.error_code = 1011;
    error.error_info = "error_2";
    error.module_name = "error_2";
    error.level = 2;
    pub_error2_msg.publish(error);
}

void RobotConfig::callback_error_msg3()
{
    static int q = 0;
    q++;
    if (q < 30 || q > 150)
    {
        return;
    }

    cti_msgs::ErrorStatus error;
    error.error_code = 102;
    error.error_info = "error_3";
    error.module_name = "error_3";
    error.level = 0;
    pub_error3_msg.publish(error);
}

//主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_config_node");
    RobotConfig rc;
    rc.init();

    if (setUnlimit() == -1)
    {
        return -1;
    }
    ros::spin();
}

//取消ndt——pose
// void RobotConfig::callback_ndt_pose(const geometry_msgs::PoseStamped &ndt_pose)
// {
//     ndt_pose_ = ndt_pose;
//     tf::StampedTransform transform_base;//定义存放变换关系的变量
//     //监听两个坐标系之间的变换
//     try{
//       tf_listen_.lookupTransform("/map","/base_link",ros::Time(0),transform_base);
//     }catch(tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//       return ;
//     }
//     base_pose_.header.stamp = ros::Time::now();
//     base_pose_.pose.position.x = transform_base.getOrigin().x();
//     base_pose_.pose.position.y = transform_base.getOrigin().y();
//     base_pose_.pose.position.z = transform_base.getOrigin().z();
//     base_pose_.pose.orientation.x = transform_base.getRotation().getX();
//     base_pose_.pose.orientation.y = transform_base.getRotation().getY();
//     base_pose_.pose.orientation.z = transform_base.getRotation().getZ();
//     base_pose_.pose.orientation.w = transform_base.getRotation().getW();
//     pub_base_pose_.publish(base_pose_);
// }

//读取文本 对比字符串
// void RobotConfig::check_mpaAndvmap_version()
// {
//     //读取vmap version版本
//     ifstream fout;
//     std::string read_path{ "/home/neousys/cti_vmap/" + run_env_ + "version_vmap"};
//     fout.open(read_path);

//     if( !fout.is_open() )
//     {
//         std::cout<<"打开"+read_path+"失败"<<std::endl;
//     }

//     std::vector<std::string> v;
//     std::string tmp;

//     while (getline(fout, tmp))
//     {
//         v.push_back(tmp);
//     }
//     fout.close();

//     //读取map version版本
//     ifstream fout2;
//     std::string read_path2{ "/home/neousys/cti_map/" + run_env_ + "version"};
//     fout2.open(read_path2);

//     if( !fout2.is_open() )
//     {
//         std::cout<<"打开"+read_path2+"失败"<<std::endl;
//     }

//     std::string tmp2;
//     getline(fout2, tmp2);
//     std::string v2 = tmp2.substr(8,5);     //获得字符串s中从第8位开始的长度为5的字符串
//     fout2.close();

//     //对比版本是否相同,相同不相同则报错
//     std_msgs::Bool flag;
//     if( v2 != v[1] )
//     {
//         flag.data = false;
//         pub_mapAvmap_ver_err.publish( flag );
//     }
//     else
//     {
//         flag.data = true;
//         pub_mapAvmap_ver_err.publish( flag );
//     }
// }

