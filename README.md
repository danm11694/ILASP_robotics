# dvrk_pegring

packages and tools for autonomous task with dvrk based on ASP and DMPs

DEPENDENCIES:
- CoppeliaSim https://www.coppeliarobotics.com/downloads (use ROS Plugin at https://github.com/lagadic/vrep_ros_bridge)
- install clingo following instructions at https://github.com/potassco/clingo with Lua option (clingo 5.3 tested in this project)
- install ILASP from ilasp.com (follow instructions for ILASP4)
- build ROS packages from https://github.com/jhu-dvrk/dvrk-ros (CISST 1.7)
- build peg_ring package from https://gitlab.com/Seirin/peg_and_ring (branch simulation for simulated task on V-REP)


DESCRIPTION:
- dvrk_task_msgs contains custom msgs (action msgs expect fields defining the action id, the agent robot, the operated object and its property, as of standard granularity definitions for actions)
- robot_control contains tools for dvrk control and motion control with DMPs
- task_reasoning contains tools for ASP management with Clingo and ILASP learning

TESTING IN SIMULATION (pegring task): 
- open scene pegring_scene.ttt in CoppeliaSim
- roslaunch robot_control dvrk_console.launch
- roslaunch peg_and_ring peg_ring.launch
- roslaunch robot_control framework.launch

NOTES:
- for real pegring, omit sections of codes marked as "V-REP..." in robot_control/scripts/motion_pegring.py, robot_control/scripts/sensing_pegring.py (maybe some tolerances in these two files need to be adjusted, e.g. for target poses and failure conditions)
- for other tasks, modify sensing_pegring.py, ILP_pegring and motion_pegring.py. In sensing, define appropriate functions for compute_fluents, compute_target and check_failure. In ILP, define appropriate functions for compute_fluents and ILASP files. In motion, define appropriate motion_policies in execution. Also modify the description file in robot_control/config/pegring.json, with definition of appropriate actions and properties (motion policy name, type either dmp or custom, and weigths for DMPs). Then, in framework.launch, change the params task_name, asp_name (for asp file in task_reasoning/asp), and the name of sensing node, and define the correct motion control file in robot_control/scripts/control.py in the code section "DEFINE MOTION CONTROL"
- When running nodes on a separate computer, dvrk_task_msgs must be built also there!
- run script reset_examples.py in task_reasoning to delete examples from previous executions in ILASP files (this should not be done normally, since previous executions of the framework should be assumed to be correct and then contribute to the task knowledge!)

