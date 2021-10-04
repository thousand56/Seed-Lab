#Repository for Seed Lab Projects
Aiden Holm, Alex Pruitt, Akshay Mungekar, Steven Dobson

The goal of this project is to have a computer vision marker location determine the rotation of a motor.

Process:
1. Computer Vision captures the frame and determines the location of the marker (NW, NE, SW, SE)
2. This is passed to an LCD display and to the arduinos
3. Arduinos determine the radial position the encoder must spin to
4. PID controller moves motor to desired location

