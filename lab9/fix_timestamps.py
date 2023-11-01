"""Convert nanosecond timestamps to seconds."""
from evo.tools import file_interface

traj = file_interface.read_tum_trajectory_file("output/orbslam/CameraTrajectory.txt")

traj.timestamps = traj.timestamps / 1e9

file_interface.write_tum_trajectory_file("output/orbslam/orb_slam3.txt", traj)

kimera_traj = file_interface.read_euroc_csv_trajectory("output/kimera/traj_vio.csv")
file_interface.write_tum_trajectory_file("output/kimera/kimera.txt", kimera_traj)
