## CITATION
@article{meli2024inductive,
  title={Inductive Learning of Robot Task Knowledge from Raw Data and Online Expert Feedback},
  author={Meli, Daniele},
  journal={Machine Learning},
  year={2024, in publication},
  publisher={Springer}
}

## DEPENDENCIES:
- tested with ROS Noetic
- CoppeliaSim 4.1 https://www.coppeliarobotics.com/downloads (use ROS Plugin at https://github.com/lagadic/vrep_ros_bridge, you can copy paste limSimExtROSInterface.so in Coppelia directory)
- install clingo following instructions at https://github.com/potassco/clingo with Lua option (clingo 5.4 tested in this project)
- install ILASP from ilasp.com (follow instructions for ILASP4)
- ROS packages from https://github.com/AndreaRoberti/dvrk_altair


## DESCRIPTION:
- dvrk_task_msgs contains custom msgs (action msgs expect fields defining the action id, the agent robot, the operated object and its property, as of standard granularity definitions for actions)
- robot_control contains tools for dvrk control and motion control with DMPs
- task_reasoning contains tools for ASP management with Clingo and ILASP learning

## TESTING IN SIMULATION (pegring task): 
- open scene pegring_scene.ttt in CoppeliaSim
- roslaunch robot_control dvrk_console.launch
- roslaunch peg_and_ring peg_ring.launch
- roslaunch robot_control framework.launch

## NOTES:
- for other tasks, modify sensing_pegring.py, ILP_pegring.py and motion_pegring.py. In sensing, define appropriate functions for compute_fluents, compute_target and check_failure. In ILP, define appropriate functions for compute_fluents and ILASP files. In motion, define appropriate motion_policies in execution. Also modify the description file in robot_control/config/pegring.json, with definition of appropriate actions and properties (motion policy name, type either dmp or custom, and weigths for DMPs). Then, in framework.launch, change the params task_name, asp_name (for asp file in task_reasoning/asp), and filenames accordingly, and define the correct motion control file in robot_control/scripts/control.py in the code section "DEFINE MOTION CONTROL"
- When running nodes on a separate computer, dvrk_task_msgs must be built also there!
- run script reset_examples.py in task_reasoning to delete examples from previous executions in ILASP files. If needed, also reset asp file (pegring_sequential.lp) and action config file (pegring.json)
- the code replicates the second part of the paper, for online ILASP for task knowledge refinement via human feedback. To generate initial rules in pegring_sequential.lp with unsupervised action identification (part 1 of the paper), just employ any action identification algorithm to your task and generate ILASP examples with noise, according to the accuracy of action classification (see Section 4.1). Example ILASP files (with results) generated with Meli and Fiorini, RAL 2021, are in folder offline_ilasp

