# Introduction
In today's fast-paced world, the integration of Unmanned Aerial Vehicles (UAVs) has marked a paradigm shift in various industries, particularly in the realm of transportation and logistics. Among these cutting-edge technologies, autonomous quadcopters have emerged as frontrunners in revolutionizing the delivery landscape. In this project, I've designed navigation algorithms for a 3-D simulated quadcopter for autonomous delivery of packages in a city environment. This project was designed for the e-Yantra Robotics Competition (eYRC), i.e. a unique annual competition launched in the month of August every year for undergraduate students in science & engineering colleges, and polytechnic. Selected teams are given a robotic kit complete with accessories and video tutorials to help them learn basic concepts in embedded systems and microcontroller programming. Abstracts of real-world problems assigned as "themes" are then implemented by the teams using the robotic kits. This competition is Hosted by IIT Bombay and sponsored by the Ministry of Education of India. 

# Understanding eDrone Model
Drones of Unmanned Aerial Vehicles (UAVs) come in two variants - fixed-wing and rotary drones. Rotary drones or multirotor drones consist of various configurations including the helicopter, four-rotor quadcopters, and six-rotor hexacopters. Each rotor type has a specific usage and is useful for specific applications. The commonly used type of multirotor is the quadcopter owing to its mechanical simplicity. In quadcopters, each motor spins in the opposite direction of the adjacent motor as shown in the figure below. This allows it to achieve vertical lift.  

![Drone](https://github.com/tusharsial/Autonomous-Delivery-Quadcopter/assets/74113713/dbb874fd-0670-44d1-a55a-2112d99ea767)

In order to maintain its pose in flight and maintain stability, a quadcopter relies heavily on sensors that constantly monitor the quadcopter's attitude. These sensors provide feedback to the quadcopter using which the flight controller makes corrections in the motor spin and thus, adjusting and shifting the quadcopter in flight so that it remains stable. 

# Path-planning & Navigation Design 
A flight controller is responsible for issuing commands to the motors as per the required motion. The flight controller relies on sensor data to generate accurate motion commands so that the UAV maintains its pose as it moves from Point A to Point B. Commonly used flight controllers in quadcopters are PixHawk, KK, and MultiWii. Two PID-based controllers namely Position Controller & Attitude Controller were designed to provide desired motion and flight stability. The following diagram represents the overall cascaded Flight Control System:   

![Controller Design](https://github.com/tusharsial/Autonomous-Delivery-Quadcopter/assets/74113713/c56264c6-2ec6-44e3-9ae6-a69bd2d922ca)

For path planning and obstacle avoidance, I leveraged the Bugs 0 algorithm. By harnessing the power of Bugs 0, the quadcopter was equipped with efficient and adaptive path-planning capabilities, enabling it to navigate through complex urban environments while avoiding obstacles. Furthermore, to ensure precise trajectory tracking, a waypoint navigation system was integrated at the controller side, allowing the quadcopter to accurately follow designated routes. 

The drone is equipped with a gripper mechanism at the bottom which can grasp objects. The gripper can be activated only when it is in close vicinity of the package. The delivery package has a QR tag on its top which contains the GPS coordinates of the final destination. I've implemented a pyzbar library-based computer vision algorithm for detecting the QR tag on delivery packages.  

# Simulation Video
The quadcopter starts from the position i.e. latitude: 19.0009248718, longitude: 71.9998318945, altitude: 22.16. The quadcopter first goes to latitude: 19.0007046575, longitude: 71.9998955286, altitude: 22.1599967919 where the delivery package is, picks up the package using the gripper mechanism, scans the QR code on the top of the package which contains the final destination where the package is to be delivered, and then finally navigates to that location. The following video demonstrates the autonomous delivery of the package via the quadcopter in a city environment. 

https://user-images.githubusercontent.com/74113713/205676430-6f412288-dd65-4a34-9c55-186bd46800fc.mp4

# Future Scope
* Use Sophisticated Path-planning algorithms like A* and Dijkstra.
* Look into adaptive and model-free control techniques (like Neural networks, Fuzzy logic, or reinforcement learning-based algorithms)
* Perform State estimation using techniques like Kalman Filter, Extended Kalman Filter, etc. 
