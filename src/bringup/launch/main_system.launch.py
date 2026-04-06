import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # <-- 이 줄이 누락되면 에러가 납니다!
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. 외부 패키지 런치 파일 경로 설정 ---
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    
    dsr_launch_path = os.path.join(
        get_package_share_directory('dsr_bringup2'), 'launch', 'dsr_bringup2_moveit.launch.py')

    # --- 2. 외부 런치 포함 (카메라 & 로봇) ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'align_depth.enable': 'true',  # 뎁스와 컬러 정렬 활성화
            'pointcloud.enable': 'true'
        }.items()
    )

    dsr_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dsr_launch_path),
        launch_arguments={
            'model': 'h2017', 
            'host': '192.168.1.200', 
            'mode': 'real'
        }.items()
    )

    # --- 3. 캘리브레이션 노드 (성아님의 YAML 기반 노드) ---
    calibration_node = Node(
        package='calibration',
        executable='tf_broadcaster',
        name='calibration_tf_node',
        output='screen'
    )
    
    vision_node = Node(
            package='vision_package',
            executable='chicken_detector',
            name='vision_node'
    )
    
    manipulation_node = Node(
            package='manipulation_package',
            executable='arm_controller',
            name='arm_node'
    )
    task_planner_node = Node(
            package='planner_package',
            executable='task_planner',
            name='planner_node',
            output='screen' # 플래너 로그를 터미널에 출력
    )
   

    # --- 4. 최종 실행 리스트 반환 ---
    return LaunchDescription([
        realsense_launch,
        dsr_robot_launch,
        calibration_node
        # 필요할 때 아래 주석을 해제하세요.
        # Node(package='vision', executable='chicken_detector', name='vision_node'),
        # Node(package='manipulation', executable='arm_controller', name='manipulation_node')
    ])
