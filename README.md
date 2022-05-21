## SUAS
The SUAS competition is designed to 
foster interest in Unmanned Aerial Systems (UAS), 
stimulate interest in UAS technologies and careers, 
and to engage students in a challenging mission.

#About the Competition
The competition requires students to design, integrate, report on, and demonstrate a UAS capable of autonomous flight and navigation, remote sensing via onboard payload sensors, and execution of a specific set of tasks. The competition has been held annually since 2002.

PATUXENT RIVER NAVAL AIR STATION (NAS):
The competition is held at the Patuxent River Naval Air Station (NAS) Webster Field in St. Mary's County, Maryland. This is the site of the UAS Test & Evaluation Directorate.

AUTONOMOUS AERIAL MISSIONS:
The competition focuses on Unmanned Aerial Systems (UAS) performing autonomous missions. Many of the tasks require autonomy to be eligible, and others receive more points for autonomy.

TECHNICAL PAPER, FLIGHT READINESS REVIEW, MISSION DEMONSTRATION:
The competition has 3 major graded components: a Technical Journal Paper which describes the systems engineering approach and the UAS design, a Flight Readiness Review (FRR) where teams describe their mission readiness and what testing gives them confidence, and a Mission Demonstration where the team is evaluated on performance.

MISSION TASKS:
The competition has a series of tasks that should be completed by the UAS system built. The competition changes these tasks year-to-year to reflect the forefront of the UAS industry. The tasks are joined to form a simulated real-world mission. Example tasks:

Interoperability - The UAS must download mission details, uploads aircraft telemetry in real time, and uploads mission deliverables to an external networked system.

Autonomous Flight - The UAS autonomously takes off, flies within boundaries, navigates a series of waypoints, and lands.

Obstacle Avoidance - The UAS autonomously avoids obstacles, which can be stationary or moving.

Object Detection - Classification, Localization. The UAS takes pictures of a search area, detects objects of interest, classifies its characteristics, and provides a GPS position.

Mapping - The UAS takes pictures of an area of interest and stitches the imagery into a map.

Air Delivery - The UAS autonomously drops a payload object so that it lands undamaged at a provided GPS position.


## Obstacle Avoidance
An optimization of objective track could effectively jump from a local minimum to reach target under a dynamic environment with moving target and obstacles. As shown in Fig. , in the proposed APF model, moving obstacles will produce a repulsion for a UAV motion, while a target location will put a gravity on the UAV motion. Both forces on the UAV will collaboratively determine the acceleration of its motion, thus control the direction towards next-step position. When the UAV falls into the local minimum, we will introduce the coordination force to overcome the local minimum dilemma. During a progress of UAV for the target location, a group of moving obstacles are present in the direction towards the target location. Here, the proposed algorithm will find a new repulsive function by incorporating the distance impact. Once UAV is relatively close to target location, the repulsive force will degrade gradually trivial up to zero.

![Screenshot 2022-05-22 034556](https://user-images.githubusercontent.com/77221967/169670737-d283584b-9b81-4f38-af7b-ec770a0bfc58.png)

## Path Planning
The  shortest  path  for  Unmanned  Aerıal  Vehicles  (UAVs)  is calculated  with  two  -dimensional  (2D)  path  planning  algorithms  in  the environment including obstacles and thus the robots could perform their tasks as  soon  as  possible  in  the  environment.  The  aim is to avoid obstacles  and  to  find  the  shortest  way  to  the  target  point.

# A* Algorithm
A* algorithm is  a  heuristics  approach algorithm.  The total cost is calculated by adding heuristics cost to the cost of the road. The  lowest  total cost way  is  preferred. So  you don't need to visit all nodes.
![Screenshot 2022-05-22 040726](https://user-images.githubusercontent.com/77221967/169671133-fe776632-09b3-4045-b865-208813354c82.png)
The function used by A* in distance calculation is as follows:  f(n) = g(n) + h(n)  (1) As it appears in the Equation 1, f (n): a heuristic function that calculates, g(n): the cost of access from the start node to the current node, h(n): the distance from the current node to the destination node is the estimated distance. 
Reference - http://users.isr.ist.utl.pt/~mir/pub/ObstacleAvoidance.pdf

