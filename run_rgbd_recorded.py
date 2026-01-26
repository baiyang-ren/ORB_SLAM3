import subprocess
import os

def run_rgbd_slam_command():
    """
    Executes the ORB_SLAM3 RGB-D command on recorded data with configurable paths.
    """
    # Configure dataset name and paths
    name = "rgbd_20260125_153627"
    input_data_base_dir = f"/home/baiyang/octa/recording_share/{name}/"

    # Define the output trajectory name (without file extension)
    trajectory_name = name

    # ORB_SLAM3 paths
    vocabulary_path = "Vocabulary/ORBvoc.txt"
    config_path = "Examples/RGB-D/RealSense_D435.yaml"
    executable_path = "./Examples/RGB-D/rgbd_euroc"

    # Check if dataset directory exists
    if not os.path.exists(input_data_base_dir):
        print(f"Error: Dataset directory not found: {input_data_base_dir}")
        return

    # Check if associations file exists
    associations_file = os.path.join(input_data_base_dir, "associations.txt")
    if not os.path.exists(associations_file):
        print(f"Error: associations.txt not found: {associations_file}")
        return

    # Construct the command as a list of arguments
    command = [
        executable_path,
        vocabulary_path,
        config_path,
        input_data_base_dir,
        associations_file,
        trajectory_name
    ]

    print("=" * 60)
    print("Running RGB-D SLAM on recorded data")
    print("=" * 60)
    print(f"Dataset: {input_data_base_dir}")
    print(f"Associations file: {associations_file}")
    print(f"Output trajectory: {trajectory_name}")
    print(f"Vocabulary: {vocabulary_path}")
    print(f"Config: {config_path}")
    print("=" * 60)
    print(f"Executing command: {' '.join(command)}")
    print("=" * 60)
    print()

    try:
        # Execute the command
        result = subprocess.run(command, check=True)
        print()
        print("=" * 60)
        print("SLAM processing completed successfully!")
        print("=" * 60)
        print("Output files:")
        print(f"  - f_{trajectory_name}.txt (camera trajectory)")
        print(f"  - kf_{trajectory_name}.txt (keyframe trajectory)")
        print("=" * 60)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        if e.stdout:
            print("STDOUT:")
            print(e.stdout)
        if e.stderr:
            print("STDERR:")
            print(e.stderr)
    except FileNotFoundError:
        print(f"Error: The executable '{command[0]}' was not found.")
        print("Please ensure ORB_SLAM3 is built and the path to 'rgbd_euroc' is correct.")
        print("Build command: cd build && make rgbd_euroc")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    run_rgbd_slam_command()
