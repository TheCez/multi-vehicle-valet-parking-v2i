#!/usr/bin/env bash

# Source this file so the selected environment and directory remain active:
#   source /home/thecez/activate_thesis.sh

echo "Select a thesis project variant:"
echo "1) Original implementation"
echo "2) All-car reachability set"
echo "3) Overlapping-obstacle strategy"
echo "4) All-car reachability set with parking-line map"
echo "5) Overlapping-obstacle strategy with final map"
echo "6) Simple strategy with final map"
echo "7) All-car reachability set with final map"
echo "8) Baseline configuration"
echo "9) Final baseline"

read -r -p "Enter a choice (1-9): " choice

case "$choice" in
  1)
    project_dir=/home/thecez/thesis/commonroad_generator3
    venv_dir=/home/thecez/thesis/commonroad_generator/.final
    carla_install=/home/thecez/CARLA_0.9.15_final
    thesis_map=Town_Valet_Parking
    ;;
  2)
    project_dir=/home/thecez/thesis/commonroad_generator2
    venv_dir=/home/thecez/thesis/commonroad_generator/.final
    carla_install=/home/thecez/CARLA_0.9.15_final
    thesis_map=Town_Valet_Parking
    ;;
  3)
    project_dir=/home/thecez/thesis/commonroad_generator
    venv_dir=/home/thecez/thesis/commonroad_generator/.final
    carla_install=/home/thecez/CARLA_0.9.15_final
    thesis_map=Town_Valet_Parking
    ;;
  4)
    project_dir=/home/thecez/thesis/commonroad_generator4
    venv_dir=/home/thecez/thesis/commonroad_generator4/.final
    carla_install=/home/thecez/CARLA_0.9.15_final_parking
    thesis_map=Town_Valet_Parking_Lines
    ;;
  5)
    project_dir=/home/thecez/thesis/commonroad_generator5
    venv_dir=/home/thecez/thesis/commonroad_generator5/.final
    carla_install=/home/thecez/CARLA_0.9.15_perfect
    thesis_map=Town_Valet_Parking_final
    ;;
  6)
    project_dir=/home/thecez/thesis/commonroad_generator6
    venv_dir=/home/thecez/thesis/commonroad_generator5/.final
    carla_install=/home/thecez/CARLA_0.9.15_perfect
    thesis_map=Town_Valet_Parking_final
    ;;
  7)
    project_dir=/home/thecez/thesis/commonroad_generator7
    venv_dir=/home/thecez/thesis/commonroad_generator5/.final
    carla_install=/home/thecez/CARLA_0.9.15_perfect
    thesis_map=Town_Valet_Parking_final
    ;;
  8)
    project_dir=/home/thecez/thesis/commonroad_baseline_config
    venv_dir=/home/thecez/thesis/commonroad_generator5/.final
    carla_install=/home/thecez/CARLA_0.9.15_perfect
    thesis_map=Town_Valet_Parking_final
    ;;
  9)
    project_dir=/home/thecez/thesis/commonroad_generator_baseline
    venv_dir=/home/thecez/thesis/commonroad_generator5/.final
    carla_install=/home/thecez/CARLA_0.9.15_perfect
    thesis_map=Town_Valet_Parking_final
    ;;
  *)
    echo "Invalid choice: $choice" >&2
    return 2 2>/dev/null || exit 2
    ;;
esac

if [[ ! -f "$venv_dir/bin/activate" ]]; then
  echo "Virtual environment not found: $venv_dir" >&2
  return 1 2>/dev/null || exit 1
fi

source "$venv_dir/bin/activate"
export CARLA_INSTALL="$carla_install"
export THESIS_PROJECT="$project_dir"
export THESIS_MAP="$thesis_map"
cd "$project_dir" || return 1 2>/dev/null || exit 1

echo "Activated: $THESIS_PROJECT"
echo "CARLA:     $CARLA_INSTALL"
echo "Map:       $THESIS_MAP"
echo "Start CARLA in another terminal with:"
echo "  \"$CARLA_INSTALL/CarlaUE4.sh\" -quality-level=Low"
