Project title: Ensuring safety in Human robot Joint actions in a dynamic environment.
#Documentation for the handover project.
Scientific Objectives:
    Motion Planning in unstructured environments amidst dynamic obstacles.
    Evaluate human safety with application to joint actions.
Software Tools:
    Ubuntu 22:04
    Cpp 17
    Python 3.8
    Ros Humble LTS
    Coppelia sim
    Media pipe

Hardware:
    UR5e
    Intel Real sense D435
    Robotiq gripper.

Project Objectives/Milestones.
    -The components and structure of a real and simulated HRC application.
    -Command robot moves using Moveit!.
    -Move the arm to a joint or Cartesian position.
    -Leverage perception capabilities including AR tag recognition and PCL.
    -Plan collision-free paths for a pick and place task.
    -Control of robot gripper.

Project structure.
    ROS Workspace
        pkgs
            moveit_related_stuff
            Camera driver
            UR driver
            Gripper driver
            Application
                Launch
                    application_setup.launch.py :initialise the setup and launch the camera and UR.
                    application_run.launch.py
                Config
                    pick_and_place_parameters.yaml
                    rviz_config.rviz
                    target_recognition_parameters.yaml
                Src
                    nodes/
                        pick_and_place_node.cpp: Main application thread. Contains all necessary headers and function calls.
                        Perception_node.cpp
                    tasks/
                        create_motion_plan.cpp
                        create_pick_moves.cpp
                        create_place_moves.cpp
                        detect_box_pick.cpp
                        initialize.cpp
                        move_to_wait_position.cpp
                        pickup_box.cpp
                        place_box.cpp
                        reset_world.cpp
                        set_attached_object.cpp
                        set_gripper.cpp

    Coppelia 
        pks    
            Scene 
                UR5 
                Sensors
                Mannequin
            Evaluation script
                Evalution metrics
                Plots

Todo:
1. Build a workcell. (with ur, gripper and camera)
2. configure workcell moveit.
3. launch workcell moveit along with the UR drivers, gripper and camera driver in application setup.
4. Write the task nodes.