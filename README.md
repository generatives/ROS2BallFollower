# ROS2 Ball Follower

This repo is a space for me to learn how to use ROS2. I am attempting to build something similar to the ball following robot I build here https://github.com/generatives/DistBot although for now I am expecting everything to stay in simulation.

This project is being developed on a Mac, with VS Code, inside a Dev Container. I had a hard time getting ROS2 to work with PyPi packages so I needed to use a venv and find a way to point ROS2 towards that venv. I did this by modifying the "setup.cfg" file in the gui package. The simulator package doesn't need extra dependencies.

I didn't want to figure out how to run turtlesim inside a Linux container on OSX so I am using the NiceGUI ROS2 examples here https://github.com/zauberzeug/nicegui/tree/main/examples/ros2 as my simulator and GUI.

The "requirements.txt" file list requirements that should be installed in your venv. You should also modify "setup.cfg" file in the gui package to point to your venv.

Here is a video of the robot following the ball in simulation
[![Following](https://github.com/user-attachments/assets/bd17b257-c46d-4ac3-b131-5e7c6ea93e30)](https://youtu.be/fqrbo736LvM "Following")
