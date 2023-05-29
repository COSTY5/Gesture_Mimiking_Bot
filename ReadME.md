# Hand Gesture Control with ROS and OpenCV

This code provides a hand gesture control system using ROS (Robot Operating System) and OpenCV (Open Source Computer Vision Library). It allows you to control a hand model in Gazebo by detecting hand gestures through a webcam.

## Prerequisites

Before running this code, make sure you have the following installed:

- Python 3
- ROS (Robot Operating System) Noetic
- OpenCV
- mediapipe
- Gazebo


## Installation

1. Download the code files to your local machine.

2. Install the required Python packages. You can use the following command to install them using pip:

   ```
   pip install rospy std_msgs opencv-python mediapipe
   ```

3. Set the correct path to the OpenCV module. In the code, there is a line that appends the path to the OpenCV module:

   ```
   sys.path.append('/path/to/opencv/module')
   ```

   Replace `/path/to/opencv/module` with the actual path to the OpenCV module on your machine.

## Usage

1. Connect your webcam to your computer.

2. Run the following command in your terminal to launch Gazebo.
   ```
   roslaunch hand_bot hand_bot.launch
   ```

3. Open a new terminal window and navigate to the directory where the code files are located.

4. Run the following command to start the hand movement control system:

   ```
   rosrun hand_bot hand_gesture.py
   ```

   The system will start capturing video from your webcam and detecting hand movements.

5. Perform hand gestures in front of the webcam to control the hand model in the simulation environment. The code detects the landmarks of your hand using the MediaPipe library, calculates the joint angles based on the detected landmarks, and publishes the joint angles to the corresponding ROS topics.

6. The hand model in the simulation environment should respond to your hand gestures according to the published joint angles.

7. Press Ctrl+C in the terminal to stop the program.


