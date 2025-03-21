import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from task_controller.gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import pandas as pd
import os

# Correct CSV file path
def get_csv_path():
    return os.path.expanduser("~/TECHIN516/my_ros2_ws/src/task_controller/task_controller/put.csv")

# Load poses dynamically from CSV
def load_poses_from_csv(file_path):
    poses = {}
    if not os.path.exists(file_path):
        print(f"❌ Error: CSV file '{file_path}' not found!")
        return {}

    df = pd.read_csv(file_path)

    # Ensure required columns exist
    required_columns = ['name', 'pos_x', 'pos_y', 'pos_z', 'x', 'y', 'z', 'w']
    if not all(col in df.columns for col in required_columns):
        print("⚠️ Warning: CSV file is missing required columns!")
        return {}

    # Convert CSV rows into Pose objects
    for _, row in df.iterrows():
        if pd.notna(row['name']):  # Ensure the pose has a valid name
            pose_name = row['name'].strip()
            poses[pose_name] = Pose(
                position=Point(
                    x=float(row['pos_x']),
                    y=float(row['pos_y']),
                    z=float(row['pos_z'])
                ),
                orientation=Quaternion(
                    x=float(row['x']),
                    y=float(row['y']),
                    z=float(row['z']),
                    w=float(row['w'])
                )
            )
    return poses

# Execute movements based on loaded poses
def execute_movements(arm, gripper, movements):
    for action in movements:
        if isinstance(action, Pose):
            success = arm.inverse_kinematic_movement(action)
            if success:
                print(f"✅ Successfully moved to: {action.position}")
            else:
                print(f"❌ Failed to move to: {action.position}")
        elif isinstance(action, float):  # Gripper movement
            gripper.move_to_position(action)
            print(f"🛠️ Gripper moved to position: {action}")
        time.sleep(1)

# Main execution
def main():
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    # Load poses from CSV
    csv_path = get_csv_path()
    poses = load_poses_from_csv(csv_path)
    if not poses:
        print("❌ No valid poses found in CSV. Exiting.")
        return

    # Define movement sequence using all available poses
    movement_sequence = [poses[pose_name] for pose_name in poses.keys() if pose_name in poses]
    movement_sequence.append(0.1)  # Open gripper
    movement_sequence.append(0.8)  # Close gripper
    movement_sequence.append(0.1)  # Release gripper

    # Execute movements (skip None values)
    execute_movements(arm, gripper, [m for m in movement_sequence if m is not None])

    # Shutdown devices
    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
