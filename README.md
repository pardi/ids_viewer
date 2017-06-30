# ids_viewer

This project aims to create a ROS node to communicate with IDS camera.
It is written in C++ and developed in two class one on top of the other, the bottom class represents the IDS camera itself and provides several tool to configurate the camera directly, instead the upper layer is responsible for the ROS communication and integration, thanks to an yaml file some configuration are editable online.
