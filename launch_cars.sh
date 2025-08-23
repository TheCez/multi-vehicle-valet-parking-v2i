#!/bin/bash

# Launch car1.py in a new terminal
gnome-terminal -- bash -c "python3 automatic_control_main_path_planning_test.py --sync; exec bash"

# Launch car2.py in another new terminal
gnome-terminal -- bash -c "python3 automatic_control_main_path_planning_test_reverse.py --sync; exec bash"

exit 0
