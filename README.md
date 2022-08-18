# Ball Tracking and Chasing Robot using ROS in Gazebo 

---
![balltracking](https://user-images.githubusercontent.com/68025565/185442505-bea3f44d-56d6-4838-9a88-56bce8469236.gif)

This project implements a simple ball tracking and chasing algorithm on a wafflebot in Gazebo. The algorithm subscribes from the `camera/rgb/image_raw` topic and then converts that image to a format accessible by `openCV`. The image processing steps are as follows:
- Set a lower and upper bound on the known RGB value of the object (in this case, red)
- Apply a mask to extract only the object of interest from the environment
- Calculate centroid of the object using the following moments formula 

  <img src="https://render.githubusercontent.com/render/math?math=C_x = \frac{M_{10}}{M_00}">
  <img src="https://render.githubusercontent.com/render/math?math=C_y = \frac{M_{01}}{M_00}">
  
  where   <img src="https://render.githubusercontent.com/render/math?math=C_x, C_y"> denotes the x and y coordinates of the centroid and   <img src="https://render.githubusercontent.com/render/math?math=M"> denotes the Moment.
  
- Calculating the x coordinate of the robot from the centroid and if it is in range,
  - Check if ball is in x range and provide linear velocity
  - If ball has a y component, then provide a proportional angular velocity along with the linear velocity to approach the ball
- If ball is not in range, rotate the robot until it is in range
- If robot is too close to the ball, stop


---
  
  
