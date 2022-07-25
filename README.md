# Boeing arm in ROS2 Foxy & Moveit2 & C++
The c++ version of Boeing ur5 arm
## Required libraries:
1. Boost
  - used for serial communication
2. Eigen3
3. Alglib 
  - used to perform cubic 2D interpolation
  - open to alternatives if better libraries are found
  - https://www.alglib.net/
4. openCV
  - used to perform matrix pseudoinverse, also gaussian filtering in mesh building

## TODOs
1. Need an end-of-file line in gcode file, otherwise the boost reading may block and enter eternal waiting
2. Add the re-timer part of the project
3. Add the mesh building part of the project
