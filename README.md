# Welcome to MTE380: PiDdy
We're a team of 5 students from the University of Waterloo tasked with creating a search and rescue robot to complete a course, via line following with computer vision, pick up a lego man and drop off the lego man in a safe zone. 

The course consists of a 6x6 grid of 30cm x 30cm squares, with a red tape path guiding the robot from the starting point to the target. A green tape box outlines two safe zones which the Lego man can be dropped off in. 

On game day, our robot was able to achieve times of 15 seconds on the first two runs, and 13 seconds in the third run. In total it weighed 131 g, mostly thanks to the mechanical and electrical design of the robot. The robot's software architecutre consisted of a Teensy MCU running the main control logic, and a Pixy camera module for the perception. 
# Software

This repo contains all of the software behind our robot, PiDdy. It specifically contains the PD controller for line following, the main state machine for feature arbitration between line following, picking up the lego man and dropping off the lego man.

## Code Standards

We are following OOP coding practices for clean, reusable code. 
