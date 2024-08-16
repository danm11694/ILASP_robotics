# INTRO
The increasing level of autonomy of robots poses challenges of trust and social acceptance, especially in human-robot interaction scenarios.
This requires an interpretable implementation of robotic cognitive capabilities, possibly based on formal methods as logics for the definition of task specifications. However, prior knowledge is often unavailable in complex realistic scenarios.
In this paper, we propose an **offline algorithm** based on **inductive logic programming from noisy examples** to extract task specifications (i.e., action preconditions, constraints and effects) directly from **raw data** of few heterogeneous (i.e., not repetitive) robotic executions. Our algorithm leverages on the output of **any unsupervised action identification algorithm from video-kinematic recordings**. Combining it with the definition of very basic, almost task-agnostic, commonsense concepts about the environment, which contribute to the interpretability of our methodology, we are able to learn logical axioms encoding preconditions of actions, as well as their effects in the event calculus paradigm. 

Since the quality of learned specifications depends mainly on the accuracy of the action identification algorithm, we also propose an **online framework for incremental refinement of task knowledge from user's feedback**, guaranteeing safe execution.

Results in a standard manipulation task and benchmark for user training in the safety-critical surgical robotic scenario (**the peg transfer**), show the robustness, data- and time-efficiency of our methodology, with promising results towards the scalability in more complex domains.

# CITATION
@article{meli2024inductive,
  title={Inductive Learning of Robot Task Knowledge from Raw Data and Online Expert Feedback},
  author={Meli, Daniele and Fiorini, Paolo},
  journal={Machine Learning},
  year={2024, in publication},
  publisher={Springer}
}

# DEPENDENCIES:
- tested with ROS Noetic
- CoppeliaSim 4.1 https://www.coppeliarobotics.com/downloads (use ROS Plugin at https://github.com/lagadic/vrep_ros_bridge, you can copy paste limSimExtROSInterface.so in Coppelia directory)
- install clingo following instructions at https://github.com/potassco/clingo with Lua option (clingo 5.4 tested in this project)
- install ILASP from ilasp.com (follow instructions for ILASP4)
- ROS packages from https://github.com/AndreaRoberti/dvrk_altair


# DESCRIPTION:
## Paper part 1: Unsupervised task knowledge learning from raw data
- offline_ilasp contains cluster.py, to generate ILASP examples and probabilistic effects of actions, starting from traces of executions
- Traces are saved in full (4 rings to be transferred), fail_test (a ring falls), blue_red (placing rings on occupied pegs). Each trace folder contains kinematic features and environmental features collected after segmentation, following
@article{meli2021unsupervised,
  title={Unsupervised identification of surgical robotic actions from small non-homogeneous datasets},
  author={Meli, Daniele and Fiorini, Paolo},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={4},
  pages={8205--8212},
  year={2021},
  publisher={IEEE}
}
- cluster.py applies kNN following the above paper, and generates ILASP examples and probabilistic effects of actions

## Paper part 2: Online supervised task knowledge refinement
- dvrk_task_msgs contains custom msgs (action msgs expect fields defining the action id, the agent robot, the operated object and its property, as of standard granularity definitions for actions)
- robot_control contains tools for dvrk control and motion control with DMPs
- task_reasoning contains tools for ASP management with Clingo and ILASP learning

# TESTING IN SIMULATION (pegring task): 
- open scene pegring_scene.ttt in CoppeliaSim
- roslaunch robot_control dvrk_console.launch
- roslaunch peg_and_ring peg_ring.launch
- roslaunch robot_control framework.launch

# NOTES for online supervised task knowledge refinement:
- for other tasks, modify sensing_pegring.py, ILP_pegring.py and motion_pegring.py. In sensing, define appropriate functions for compute_fluents, compute_target and check_failure. In ILP, define appropriate functions for compute_fluents and ILASP files. In motion, define appropriate motion_policies in execution. Also modify the description file in robot_control/config/pegring.json, with definition of appropriate actions and properties (motion policy name, type either dmp or custom, and weigths for DMPs). Then, in framework.launch, change the params task_name, asp_name (for asp file in task_reasoning/asp), and filenames accordingly, and define the correct motion control file in robot_control/scripts/control.py in the code section "DEFINE MOTION CONTROL"
- When running nodes on a separate computer, dvrk_task_msgs must be built also there!
- run script reset_examples.py in task_reasoning to delete examples from previous executions in ILASP files. If needed, also reset asp file (pegring_sequential.lp) and action config file (pegring.json)

