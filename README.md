# NYCU Robotic Project 1

This topic provides mutual verification of forward and inverse kinematics

**Task1. forward  kinematics**

When you input a array [20 20 20 20 20 20 ]  which represent six angles, the command window will show joint variable.

```
[n o a p]:
    0.1058   -0.6425    0.7589    0.5776
    0.7019    0.5889    0.4007    0.3688
   -0.7044    0.4903    0.5133    0.1968
         0         0         0    1.0000

output:
    0.5776    0.3688    0.1968   34.8424   59.1189   27.8338
```

**Task2.  inverse kinematics**

when you input a matrix 

[0.1058   -0.6425    0.7589    0.5776;
0.7019    0.5889    0.4007    0.3688;
-0.7044    0.4903    0.5133    0.1968;
0         0         0    1.0000;] which represent the  joint variable , we can get six angles.

```
Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
   20.0007   20.0041   19.9897   19.9957   20.0025   20.0074

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta2 is out of range!
 -134.8839 -127.2069   19.9897   19.1820   50.8906 -166.6135

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta3 is out of range!
theta4 is out of range!
   20.0007  -52.7931  165.2995  171.6782   53.9171 -136.1984

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta2 is out of range!
theta3 is out of range!
theta4 is out of range!
 -134.8839 -200.0041  165.2995  146.2406   27.3089   56.4731

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta4 is out of range!
   20.0007   20.0041   19.9897 -160.0043  -20.0025 -159.9926

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta2 is out of range!
theta4 is out of range!
 -134.8839 -127.2069   19.9897 -160.8180  -50.8906   13.3865

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta3 is out of range!
   20.0007  -52.7931  165.2995   -8.3218  -53.9171   43.8016

Corresponding variable (theta1, theta2, theta3, theta4, theta5, theta6, theta7, theta8) 
theta2 is out of range!
theta3 is out of range!
 -134.8839 -200.0041  165.2995  -33.7594  -27.3089 -123.5269
```

To get more informations, please see report **Robotics_Project1_312512061.pdf.**  We include the mathematical derivation.
