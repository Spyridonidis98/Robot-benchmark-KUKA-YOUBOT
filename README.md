# Robot benchmark KUKA YOUBOT

![Screenshot (43)](https://user-images.githubusercontent.com/79518257/195626435-3227e82a-1021-415a-a6fb-7f7acc73efaf.png)

The objective is to pick a cube and move it to a target position. To do  this we first have to solve the forward kinematics of the robot to be able to locate the position of the end effector in world.

![Untitled2](https://user-images.githubusercontent.com/79518257/195626826-5c8977ed-ecfa-44c8-9f43-66d58bc4cfc6.png)

# World to base transform 
Because the robot is not stationery but moves, we have to find the transform table that will take as from the world axis base of the robot. The transorm table is shown below.

![Screenshot (45)](https://user-images.githubusercontent.com/79518257/195628006-d343fa5a-fec7-4841-b3f4-7a1658213bf0.png)

# DH Table for Kuka Youbot manipulator
Because the starting position of the arm, which is completely stretched and pointing up we have to add pi/2 to θ2 and -pi/2 to θ3.

![Screenshot (46)](https://user-images.githubusercontent.com/79518257/195628383-1a26b282-1001-4842-9793-4ddd31469a74.png)

#Moving the base 
The base can move in x,z directions and rotate around it axes. So depending on  how to we want to move we have to rotate the wheels in different configurations as shown in figure below. 

![Screenshot (48)](https://user-images.githubusercontent.com/79518257/195628701-f353e63e-67f9-4fe1-a852-e3991fb784d6.png)

To move 1 meter we have to calculate how many rads we have to rotate the wheels which depends on the wheels radius which is 5cm. To do that we simply divide by the wheels circumference and multiply by 2 pi which gives as exactly 20 . When we move side ways we have to take in consideration that we will move a smaller distance, which is cos(45)=0.707% smaller  for the same rotations compared to moving front. So we have to move our wheels 1/0.707 = 1,414 time more when moving side ways. This parameter has only to with the way the wheels are designed and change depending on the sleep of the surface that the robot moves on, for the simulation we found that 1,444 is the right number. The same goes for the rotation which has to be multiplied by 10,35.

# Jacobian and reverse kinematics 
Using the tables from forward kinematics we calculate the Jacobian of the manipulator which we are going to use to calculate the pseudo inverse  J^+. By multiplying the J^+ by the velocities that we want to achieve on the end effector u_e we get the velocities q ̇ that we have to apply to the joints of manipulator to achieve the desired u_e. By using this method we are going to calculate the vector e ⃗ to our desired target and multiply it by J^+ to make a small step towards our desired goal, recalculate  J^+ and keep repeating the same process until we reach our goal.
When the robot starts moving it accelerates rapidly because it speed it’s proportional to e ⃗ and some slip between the wheels and the surface occurs. Because we update the position of robot in space by the speed of rotation of it’s wheels and for how long they been rotating , if the wheels slip we will have the wrong position of the robot in space. To overcome this we accelerate the wheels letting them to get bigger speed over time instead of letting them get an instant speed so no slip will occur. 

# Mission planning
We are going to move the base to the destination of the cube  and then grab the cube rotate 90 degrees, go to the point that the cube has to be placed and release the cube.

https://user-images.githubusercontent.com/79518257/195633254-693c03ce-035f-4609-bfba-0c1d552961d7.mp4







