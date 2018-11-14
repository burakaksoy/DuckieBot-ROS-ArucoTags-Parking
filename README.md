# DuckieBot

This repository is created for one of the DuckieBot projects at RPI Robotics1 course.  

In this project, Duckiebots will perform several parking scenarios using single camera visual feedback, which utilizes the guidance of track lines and squared fiducial markers. Squared Fiducial Markers will be used to calculate the relative pose of other Duckiebots also present in the scene, while track lines will be used for guidance or general helper marks. 

Helpful links to start:

- https://www.duckietown.org/
- https://www.duckietown.org/instructors/classes/educational-resources
- http://docs.duckietown.org/
- https://github.com/duckietown/Software
- Duckiebot [setup guide](https://docs.google.com/document/d/1J0EYZkcoZcbjndBVh_g7ouEWFcPu9WZ6be_LVghDoII/edit)
  
- https://docs.opencv.org/3.4.2/d5/dae/tutorial_aruco_detection.html
- https://www.uco.es/investiga/grupos/ava/node/26
- Main Paper: https://www.sciencedirect.com/science/article/pii/S0262885618300799
- https://www.youtube.com/watch?v=VsIMl8O_F1w

Project proposal due: 9/21
Project report/video/software due: 12/17

# Proposal
[Proposal Link](https://docs.google.com/document/d/1Dqug7z7aldNEKwI-MgvFbbHySeMAW2DbE9ogw2JG2vc/edit?usp=sharing)  
The proposal includes the following information:
- Proposed project (based on the assigned project type)
- Summary of project 
- Background and motivation of the project (include references if appropriate).
- Scope (see below – must include the proposed skill demonstration, but it may be modified as the semester progresses).  Include project goals and classify them as baseline, target, and reach goals.
- Division of work in the team
- Assessment of confidence level/risk (skill level of team members, new skills to acquire, part requirement, etc.).
- Addition parts/components that will need to be purchased
- Key intermediate steps and timeline
- Update project diary at least once a week (diary link will be send to each group)

## Timeline
-[09/25/2018]: 
  Actions Completed
  - Global Settings,SSH Keys, Clone Duckietown Repository
  - Setup ROS environment
  - Soldering the Pi Hat Circuit Boards
  - Robot Assembly
  
-[10/10/2018]:
  - GitHub issue resolved with TA. GitHub is not really required.
  - Battery replaced because it was too big in size and did not fit to robot.
  - Started joystick manipulation from Pi
  
-[10/19/2018]:
  - Could not control bot using Joystick through remote connection.
  - It was an ssh issue. Solved.
  - Remote connection was established properly. Had to add bot IP as well as vehicle's IP to /etc/hosts in own machine and        machine's IP to bot's /etc/hosts
  
-[10/27/2018]:
  - Camera calibration is done.
  
-[11/3/2018]:
  -Issues with wheel calibration. Omni wheel getting stuck randomly, causing different drift directions. Trying after application of grease to omniwheel.
  
-[11/11/2018]:
  - CALIBRATION
  
  I made my experiments for 2 meters long line.
  
  -- TRIM PARAMETER (T) CALIBRATION
  
  I get the best results with T=+0.012 value with command:
  rosservice call /duckiepark/inverse_kinematics_node/set_trim  0.012

  - One important thing that I have noticed is that, when the move command is towards backwards robot moves almost perfectly    in the path without drifting. However when going forwards robot might have random driftings towards left or right.
  
  Therefore changing the directions of the robot might be a good idea for more consistent paths. (But this will also require putting the camera to back instead of front.)

  -- GAIN PARAMETER (G) CALIBRATION
  - If G is higher than 1.00 robot might put its less heavy side to up because of the too much acceleration. Therefore G=1.00 with command:
  - rosservice call /duckiepark/inverse_kinematics_node/set_gain 1.00

  - Set parameters are saved using the command:
  - rosservice call /duckiepark/inverse_kinematics_node/save_calibration
  
-[11.12.2018]:
 Learning ROS to write our own codes to Duckie
 - Starting from last week, I completed reading a book named "Gentle Introduction to ROS". 

-[11.13.2018]:
 Line Detection Demo
 - In a new window start the lane filter node:
	 $roslaunch duckietown lane_filter.launch veh:=duckiepark
 - This command gave several errors and ended. I applied the trouble shooting steps.
 - As stated in the document it Takes hours to complete the installation.
 - After more than 3 hours waiting for installation of "sckit-learn" system gave an error which states no space left on device! Here is the error:
 /tmp/ccVKRMBr.s: Fatal error: can't close build/temp.linux-armv7l-2.7/scipy/sparse/sparsetools/bsr.o: No space left on device
 - An e-mail has been sent to TA.
 - UPDATE: After one more try with the second suggested command in the documentaion, everything worked successfully!
 - Now we are able to run the line Detection Demo
 
 -[11.13.2018]
  -Successfully operated webcam using ROS. Will apply same method for pose estimation.
 
 
 
