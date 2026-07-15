from launch import LaunchDescription                                                                                                                                                      
from launch_ros.actions import Node                                                                                                                                                       
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Объявление аргументов
    iam_token_arg = DeclareLaunchArgument(
        'iam_token',
        default_value=os.environ.get('IAM_TOKEN', ''),
        description='IAM token for Yandex Cloud'
    )

    folder_id_arg = DeclareLaunchArgument(
        'folder_id',
        default_value=os.environ.get('FOLDER_ID', ''),
        description='Folder ID for Yandex Cloud (required for user account)'
    )

    set_pythonpath = SetEnvironmentVariable(
        'PYTHONPATH',
        value='/root/venv/lib/python3.12/site-packages' + os.pathsep + os.environ.get('PYTHONPATH', '')
    )

    return LaunchDescription([
        iam_token_arg,
        folder_id_arg,
        set_pythonpath,

        # STT Node
        Node(
            package='voice',
            executable='stt_node',
            name='stt',
            parameters=[{
                'iam_token': LaunchConfiguration('iam_token'),
                'folder_id': LaunchConfiguration('folder_id'),
                'language': 'ru-RU',
                'sample_rate': 8000
            }],
            output='screen'
        ),

        # TTS Node
        Node(
            package='voice',
            executable='tts_node',
            name='tts',
            parameters=[{
                'iam_token': LaunchConfiguration('iam_token'),
                'folder_id': LaunchConfiguration('folder_id'),
                'voice': 'zahar',
                'speed': 1.0
            }],
            output='screen'
        )
    ])
