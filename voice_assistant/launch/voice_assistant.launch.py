from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='voice_assistant',
            executable='speaker_node',
            output='screen',
            parameters=[
                {'sync_mode': True},
            ]
        ),

        Node(
            package='voice_assistant',
            executable='nlp_node',
            output='screen',
        ),

        Node(
            package='voice_assistant',
            executable='mic_node',
            output='screen',
            parameters=[
                {'language': 'en-US'},
                {'keyword': 'Chloe'},
            ],
        ),
    ])
