from launch import LaunchDescription
import launch_ros.actions
import ament_index_python
import os


def generate_launch_description():
    cti_file_path = os.environ['HOME']
    cti_vehicle_config_file_path = os.environ['CTI_ROBOT_CONFIG_PATH'] + "/vehicle.yaml"
    cti_static_tf_config_file_path = ament_index_python.get_package_share_directory("robot_config") + "/config/static_tf.yaml"
    cti_topic_config_file_path = os.environ['CTI_TF_PATH'] + "/topic_state.yaml"
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_config', 
            executable='robot_config_node', 
            parameters=[
              {'CTI_FILE_PATH' : cti_file_path},
              {'CTI_VEHICLE_CONFIG_FILE_PATH' : cti_vehicle_config_file_path},
              {'CTI_TOPIC_CONFIG_FILE_PATH' : cti_topic_config_file_path},
              {'CTI_STATIC_TF_CONFIG_FILE_PATH' : cti_static_tf_config_file_path},
            ],
            output='screen'
        ),
    ])