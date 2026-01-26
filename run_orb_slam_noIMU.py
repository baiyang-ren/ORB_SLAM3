import subprocess
import os
import shutil

def run_orb_slam_command():
    """
    Executes the ORB_SLAM3 stereo command and moves trajectory files to output directory.
    """
    name = "20260125_183432"
    input_data_base_dir = f"/home/baiyang/octa/recording_share/dataset_{name}/"

    # Define the output directory for trajectory files. CHANGE THIS to your desired location.
    output_trajectory_dir = f"/home/baiyang/octa/recording_share/dataset_{name}/"

    # Trajectory name (5th argument for stereo_euroc)
    trajectory_name = f"dataset_{name}"

    # Ensure the output directory exists
    os.makedirs(output_trajectory_dir, exist_ok=True)

    # Construct the command as a list of arguments
    command = [
        "./Examples/Stereo/stereo_euroc",
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo/RealSense_D435.yaml",
        input_data_base_dir,
        os.path.join(input_data_base_dir, "mav0", "timestamps.txt"),
        trajectory_name  # Add trajectory name as 5th argument
    ]
    print(f"Executing command: {' '.join(command)}")
    print(f"Output files will be moved to: {output_trajectory_dir}")

    try:
        # Execute the command (runs from /home/baiyang/octa/ORB_SLAM3)
        # Don't use check=True because ORB-SLAM3 often crashes after saving trajectories
        result = subprocess.run(command)

        if result.returncode == 0:
            print("Command executed successfully.")
        else:
            print(f"Command exited with code {result.returncode} (this is normal for ORB-SLAM3)")

    except FileNotFoundError:
        print(f"Error: The executable '{command[0]}' was not found.")
        print("Please ensure ORB_SLAM3 is built and the path to 'stereo_euroc' is correct.")
        return
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return

    # Always attempt to move trajectory files, even if command crashed
    # (ORB-SLAM3 usually saves files before crashing)
    print("\nChecking for trajectory files...")
    orb_slam3_dir = "/home/baiyang/octa/ORB_SLAM3"
    kf_file = f"kf_{trajectory_name}.txt"
    f_file = f"f_{trajectory_name}.txt"

    kf_src = os.path.join(orb_slam3_dir, kf_file)
    f_src = os.path.join(orb_slam3_dir, f_file)
    kf_dst = os.path.join(output_trajectory_dir, kf_file)
    f_dst = os.path.join(output_trajectory_dir, f_file)

    files_moved = 0
    if os.path.exists(kf_src):
        shutil.move(kf_src, kf_dst)
        print(f"✓ Moved {kf_file} to {output_trajectory_dir}")
        files_moved += 1
    else:
        print(f"✗ Warning: {kf_file} not found in {orb_slam3_dir}")

    if os.path.exists(f_src):
        shutil.move(f_src, f_dst)
        print(f"✓ Moved {f_file} to {output_trajectory_dir}")
        files_moved += 1
    else:
        print(f"✗ Warning: {f_file} not found in {orb_slam3_dir}")

    if files_moved == 2:
        print(f"\n✓ Successfully saved {files_moved} trajectory files to {output_trajectory_dir}")
    elif files_moved > 0:
        print(f"\n⚠ Partially successful: {files_moved}/2 trajectory files moved")
    else:
        print(f"\n✗ No trajectory files were found to move")

if __name__ == "__main__":
    run_orb_slam_command()
