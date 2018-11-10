# DuckieBot

This repository is created for one of the DuckieBot projects at RPI Robotics1 course.  

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
[Proposal Draft Link](https://docs.google.com/document/d/1Dqug7z7aldNEKwI-MgvFbbHySeMAW2DbE9ogw2JG2vc/edit?usp=sharing)  
The proposal should including the following information:
- Proposed project (based on the assigned project type)
- Summary of project 
- Background and motivation of the project (include references if appropriate).
- Scope (see below – must include the proposed skill demonstration, but it may be modified as the semester progresses).  Include project goals and classify them as baseline, target, and reach goals.
- Division of work in the team
- Assessment of confidence level/risk (skill level of team members, new skills to acquire, part requirement, etc.).
- Addition parts/components that will need to be purchased
- Key intermediate steps and timeline
- Update project diary at least once a week (diary link will be send to each group)

-- Create proposal google doc.
-- Create timetable google doc.
-- Start to write the proposal.
-- Find a paper for autonomous parking.
-- Add Nemo to github for this project.

## Key steps:
- [ ] Duckiebot setup (may include soldering) 
- [ ] Wheels and camera calibration 
- [ ] Duckietown construction
- [ ] Path planning (open loop control): Use Duckietown map to drive Duckiebot from one location to another, including all known constraints
- [ ] Path following (vision guided feedback control)
- [ ] New skill or refinement of existing skill (not already in the MIT repository) Examples: navigation based on signs, collision avoidance/braking/slow-down/stop, localization based on known landmarks, platooning, parking, tractor-trailer, working together with Dobot.

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
