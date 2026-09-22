from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to the RTAB-Map database to save to / resume from'
    )

    delete_db_on_start_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Set true to start a fresh map, false to resume from database_path'
    )

    initial_pose_arg = DeclareLaunchArgument(
        'initial_pose',
        default_value='',
        description='Initial map->odom pose guess on resume: "x y z roll pitch yaw"'
    )

    common_parameters = [{
        'database_path': LaunchConfiguration('database_path'),

        # Frames
        'frame_id': 'simple_drone/base_footprint',
        'odom_frame_id': 'simple_drone/odom',
        'map_frame_id': 'map',
        'wait_imu_to_init': True,
        'wait_for_transform': 3.0,
        #'wait_for_transform_duration': 3.0,
        'Odom/Strategy': '0',
        'Odom/ResetCountdown': '1',
        'publish_tf': True, # Keep TF streaming healthy to avoid map drops

        # RGB-D processing
        'subscribe_depth': False,
        'subscribe_rgb': True,
        # LiDAR SLAM
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        # Odometry
        'subscribe_odom': True,

        'use_sim_time': True,
        
        # --- FIX 1: Decrease sync buffers to process ONLY fresh live frames ---
        'sync_queue_size': 16,
        'topic_queue_size': 16,

        # SLAM
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityByTime': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.1', # Create map keys more frequently for rotation stability
        'RGBD/LinearUpdate': '0.1',   # Create map keys more frequently for translation stability
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '4.0',
        'RGBD/MaxOdomCacheSize': "0",

        'Grid/FromDepth': 'false',
        'Grid/3D': 'true',
        'Grid/From3D': 'true',
        'Grid/Sensor': '0',
        'Grid/RangeMax': '10.0',
        'Grid/NormalsSegmentation': 'false', # Disable heavy surface math inside smooth mazes
        'Grid/RayTracing': 'true',
        #'Grid/MaxGroundHeight': '0.15',
        'Grid/MaxObstacleHeight': '10.0',
        'Grid/CellSize': '0.15', # Slightly step up voxel grid sizes to clear up CPU latency

        # --- FIX 2: Relax ICP constraints for flat corridor/maze worlds ---
        'Reg/Strategy': '1', 
        'Icp/VoxelSize': '0.2',
        'Icp/PointToPlane': 'false', # 🌟 CHANGE TO FALSE: PointToPoint handles flat corridors reliably
        'Icp/PointToPlaneMinComplexity': '0.001', # Minimize early loop drop thresholds
        'Icp/CorrespondenceRatio': '0.2', # Lower criteria threshold to ensure links don't break early
        'Icp/Iterations': '10', # Double processing attempts to lock positions better
        'Icp/Epsilon': '0.001',
        'Icp/MaxCorrespondenceDistance': '0.2',
        'Icp/MaxTranslation': '0.5', # Cap max translations to block massive erratic jumps
        'Icp/MaxRotation': '0.78',    # Cap max rotational estimation drift 
        'Icp/DownsamplingStep': '6',
        
        # --- FIX 3: Control CPU processing rates ---
        'Rtabmap/DetectionRate': '0.5', # 🌟 CHANGE TO 1.0: Forces a healthy processing loop rate of 1Hz
        'approx_sync': True,
        'approx_sync_max_interval': '1.0',
        
        # 2D SLAM (If drone flies at a fixed height, turn these True to make mapping trivial)
        'Optimizer/Slam2D': 'false',
        'Reg/Force3DoF': 'false',
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.2',
        'Rtabmap/TimeThr': '1000',
        'Kp/MaxFeatures': '400',

        'Rtabmap/TimeThr': '1500', # Force optimization updates to stay under 800ms
        'Mem/RehearsalSimilarity': '0.30',
        'Mem/STMSize': '10', 
        'Mem/WorkingMemSize': '100',
        'Mem/IncrementalMemory': 'false'
    }]

    remappings = [
        ('odom', '/simple_drone/odom'),
        ('scan_cloud', '/simple_drone/laser_scanner/out'),
        ('imu', '/simple_drone/imu/out'),
        ('gps/fix', '/simple_drone/gps/nav'),
        ('rgb/image', '/simple_drone/front/image_raw'),
        ('rgb/camera_info', '/simple_drone/front/camera_info'),
    ]

    rtabmap_fresh = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        namespace='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        parameters=common_parameters,
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration('delete_db_on_start')),
    )

    resume_parameters = common_parameters + [{
        'initial_pose': LaunchConfiguration('initial_pose'),
    }]

    rtabmap_resume = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        namespace='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=resume_parameters,
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration('delete_db_on_start')),
    )

    return LaunchDescription([
        database_path_arg,
        delete_db_on_start_arg,
        initial_pose_arg,
        rtabmap_fresh,
        rtabmap_resume
    ])
