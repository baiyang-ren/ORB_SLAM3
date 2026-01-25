import subprocess
import os

def run_orb_slam_command():
    """
    Executes the ORB_SLAM3 stereo-inertial command with a configurable output directory.
    """
    name = "20260125_101802"
    input_data_base_dir = f"/home/jetson/octa/recording_share/dataset_{name}/"
    # Define the output data directory. Change this variable to point to your desired output.
    output_data_base_dir = f"/home/jetson/octa/recording_share/dataset_{name}/"

    # Ensure the output directory exists
    os.makedirs(output_data_base_dir, exist_ok=True)
    os.makedirs(os.path.join(output_data_base_dir, 'mav0'), exist_ok=True)

    # Construct the command as a list of arguments

    command = [
        "./Examples/Stereo/stereo_euroc",
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo/RealSense_D435.yaml",
        output_data_base_dir,
        os.path.join(output_data_base_dir, "mav0", "timestamps.txt")
    ]
    print(f"Executing command: {' '.join(command)}")

    try:
        # Execute the command
        result = subprocess.run(command, check=True)
        print("Command executed successfully, output streamed to console.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
        print("STDOUT:")
        print(e.stdout)
        print("STDERR:")
        print(e.stderr)
    except FileNotFoundError:
        print(f"Error: The executable '{command[0]}' was not found.")
        print("Please ensure ORB_SLAM3 is built and the path to 'stereo_inertial_euroc' is correct.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    run_orb_slam_command()
