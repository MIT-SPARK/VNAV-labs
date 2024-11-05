#!/bin/bash
###################################################################
# Fill the variables below

# Specify path of the EuRoC dataset.
# The path can be absolute, or relative to this file location.
DATASET_PATH="/datasets/MH_01_easy"

# Specify: 0 to run on EuRoC data, 1 to run on Kitti (not supported)
DATASET_TYPE=0

# Specify: 1 to enable the LoopClosureDetector, 0 to not.
USE_LCD=0

# Specify: 1 to enable logging of output files, 0 to not.
LOG_OUTPUT=1
###################################################################

###################################################################
# Other PATHS
# All paths can be absolute or relative to this file location.

# Build path: specify where the executable for Kimera is.
BUILD_PATH="build"

# Params path: specify where the parameters for Kimera are.
PARAMS_PATH="/kimera_params/"

# Vocabulary path: specify where the vocabulary for loop closure is.
VOCABULARY_PATH="vocabulary"

# Output path: specify where the output logs will be written.
# (only used if LOG_OUTPUT is enabled)
OUTPUT_PATH="/output/kimera/"
###################################################################

# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Using dataset at path: $DATASET_PATH"
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) DATASET_PATH=$2
          echo "Using dataset at path: $DATASET_PATH"
          shift ;;
        # Option -d, set dataset type
      -d) DATASET_TYPE=$2
          echo "Using dataset type: $DATASET_TYPE"
          echo "0 is for euroc and 1 is for kitti"
          shift ;;
        # Option -p, provides path to dataset.
      -c) PARAMS_PATH=$2
          echo "Using params at path: $PARAMS_PATH"
          shift ;;
      -lcd) USE_LCD=1
           echo "Run VIO with LoopClosureDetector!" ;;
      -log) LOG_OUTPUT=1
           echo "Logging output!";;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

cd Kimera-VIO

echo """ Launching:

            ██╗  ██╗██╗███╗   ███╗███████╗██████╗  █████╗
            ██║ ██╔╝██║████╗ ████║██╔════╝██╔══██╗██╔══██╗
            █████╔╝ ██║██╔████╔██║█████╗  ██████╔╝███████║
            ██╔═██╗ ██║██║╚██╔╝██║██╔══╝  ██╔══██╗██╔══██║
            ██║  ██╗██║██║ ╚═╝ ██║███████╗██║  ██║██║  ██║
            ╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝

 """

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

$BUILD_PATH/stereoVIOEuroc \
  --dataset_type="$DATASET_TYPE" \
  --dataset_path="$DATASET_PATH" \
  --initial_k=50 \
  --final_k=10000 \
  --params_folder_path="$PARAMS_PATH" \
  --use_lcd="$USE_LCD" \
  --vocabulary_path="$VOCABULARY_PATH/ORBvoc.yml" \
  --flagfile="$PARAMS_PATH/flags/stereoVIOEuroc.flags" \
  --flagfile="$PARAMS_PATH/flags/Mesher.flags" \
  --flagfile="$PARAMS_PATH/flags/VioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/RegularVioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" \
  --logtostderr=1 \
  --colorlogtostderr=1 \
  --log_prefix=1 \
  --v=0 \
  --vmodule=Pipeline*=00 \
  --log_output="$LOG_OUTPUT" \
  --log_euroc_gt_data="$LOG_OUTPUT" \
  --save_frontend_images=1 \
  --visualize_frontend_images=1 \
  --output_path="$OUTPUT_PATH"
