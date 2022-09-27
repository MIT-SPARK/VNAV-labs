from evo.tools import file_interface

traj = file_interface.read_tum_trajectory_file("f_dataset-MH01_stereo.txt")

traj.timestamps = traj.timestamps / 1e9

file_interface.write_tum_trajectory_file("f_dataset-MH01_stereo_fixed.txt", traj)
