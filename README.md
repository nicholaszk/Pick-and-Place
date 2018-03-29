# Pick-and-Place
  
Applying inverse kinematics transforms to calculate joint angles for a simulated arm to drop a designated (picked) object into a stationary bin  
  
This project is part of Udacity's Robotics Software Engineering Nanodegree coursework. Outside of the defined steps taken in this README, it can be assumed that other code belongs to Udacity.  
  
## Project: Kinematics Pick & Place
---

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./completedplacement.JPG
[image2]: ./DHcalcs.JPG
[image3]: ./inverseOrientationCalcs.JPG

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This is my writeup.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The included xacro file basically has all the information we need (joints, length of links between joints, etc.) to find our DH parameters. For each link (including the base link between the ground and joint 1), we need 4 different parameters: twist angle (angle between z axes), link length, link offest, and joint angle.  
  
The relevant Udacity lesson defines the four parameter names as follows:  
Twist angle (alpha<sub>i-1</sub>) = angle between Z<sub>i-1</sub> and Z<sub>i</sub> measured about X<sub>i-1</sub> in a right-hand sense  
Link length (a<sub>i-1</sub>) = distance from Z<sub>i-1</sub> to Z<sub>i</sub> measured along X<sub>i-1</sub> where X<sub>i-1</sub> is perpendicular to both Z<sub>i-1</sub> and Z<sub>i</sub>  
Link offset ( d<sub>i</sub>) = signed distance from X<sub>i-1</sub> to X<sub>i</sub> measured along Z<sub>i</sub>  
Joint angle (Theta<sub>i</sub>) = angle between X<sub>i-1</sub> and X<sub>i</sub> measured about Z<sub>i</sub> in a right-hand sense  
  
I have included the following figure to model the KR210 joints and illustrate the criteria for DH parameter calculations:  
![alt text][image2]  

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -.054 | 1.5 | q4
4->5 |  pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.  
  
The transformation matrix used to move from one link's frame of reference to the next is shown in the following python function:  
  
```python


def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                 0,                 0,           0,             1]])
    return TF
```

The following code provides examples of how individual and generalized transforms were created:  
```python
#individual transforms
T0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)  
T1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)  
  
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)  
```
  
```python
#general transform
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The provided code from Udacity extracts the yaw, pitch, roll, and location of the end effector from the request. We are left with the postion of the wrist center after the following code:  

```python
#calculating the wrist center coordinates
WC = EE - (0.303) * ROT_EE[:,2]
```  

To find theta1, we simply takes the arctangent of the x and y ground coordinates of the WC as follows:

```python
#calculate theta1
theta1 = atan2(WC[1], WC[0])
```

Theta2 and theta3 are also very straight forward in concept. Using the coordinates of the WC and basic trigonometry, we can find the distance between the base (the end of 90 degree ELL piece) and the WC. Combining that distance with two link lengths, we have a triangle with all 3 side lengths known. Therefore, we can use cosine law for SSS triangles to calculate all 3 interior angles, ultimately leading us to the exterior angles that will become theta2 and theta3. The walkthough also brings up the point of sag in link 4 and presents a solution for mitigating. The python code is as follows:   
  
```python
# SSS triangle for theta2 and theta3
    	side_a = 1.501
    	side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
    	side_c = 1.25

    	angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
    	angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
    	angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

      theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
      theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m
 ```
To find the remaining theta angles, we need to find the rotation matrix <sup>3</sup><sub>6</sub>R. We find <sup>3</sup><sub>6</sub>R by mutliplying <sup>0</sup><sub>3</sub>R<sup>-1</sup> by <sup>0</sup><sub>6</sub>R. Once we have <sup>3</sup><sub>6</sub>R, we can evaluate <sup>3</sup><sub>6</sub>R symbolically using only the unknown variables theta4, theta5, and theta6. Then I followed the process shown below:  
![alt text][image3]  
  

```python
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

        R3_6 = R0_3.inv("LU") * ROT_EE

	    # Euler angles from rotation matrix
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 
  
I have included a screenshot of an object successfully dropped in the bin using my IK_sever.py code.
  
![alt text][image1]
