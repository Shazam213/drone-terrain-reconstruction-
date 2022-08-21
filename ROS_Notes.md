## ROS 
### Core ROS (Beginner/Intermediate)
* Robot Operating System (ROS or ros) is an open-source robotics middleware suite which is _not_ an operating system (OS) but a **set of software frameworks** for robot software development.<br>

* File Structure in ROS:
![image](https://user-images.githubusercontent.com/95737452/185779438-c5aecf13-618c-4cf0-9514-bce020a7c164.png)<br>
![image](https://user-images.githubusercontent.com/95737452/185779548-405c58b5-c540-433a-aa40-7f8a71e7cbaa.png)<br>
![image](https://user-images.githubusercontent.com/95737452/185779575-c1294c6a-3a0c-4fc8-8bd1-056499bf2467.png)<br>![image](https://user-images.githubusercontent.com/95737452/185779808-be79f613-d918-4ac3-8f54-f8ac0b3a892a.png)<br>

* Commands:
  * rospack: Allows you to get information about packages. The _find_ option returns the path to package. <br> ![image](https://user-images.githubusercontent.com/95737452/185779644-14c6f2ae-d6ff-45fd-8e7b-10bbd00236c6.png)<br>
  * roscd: Allows you to change directory (cd) directly to a package or a stack.<br>![image](https://user-images.githubusercontent.com/95737452/185779668-c53d9f7c-9e95-4f35-ae75-207e588a4b57.png)<br>
  * rosls: Allows you to ls (list items) directly in a package by name rather than by absolute path.<br>![image](https://user-images.githubusercontent.com/95737452/185779698-ee002425-3320-4b15-b3b7-7b05ec4f31f9.png)<br>
  * Pressing **TAB** after certain commands allows you to fill out the rest of it, comes in handy for tedious commands.
 
* Catkin Packages:
  * For a package to be considered a catkin package it must meet a few requirements:<br>
  ![image](https://user-images.githubusercontent.com/95737452/185779850-9df15dc2-581a-47ba-adda-f71fa7153e93.png)
  * Creating a Catkin Package: catkin_create_pkg script is used to create a catkin workspace, as follows:<br>![image](https://user-images.githubusercontent.com/95737452/185779928-6bb87f53-bfd8-46c1-b39d-ad3905c58936.png)<br>![image](https://user-images.githubusercontent.com/95737452/185779943-bf50c46f-6614-4fc9-929c-3434a250749f.png)
  * Building & Sourcing: <br>![image](https://user-images.githubusercontent.com/95737452/185780048-ce6e7dd0-070a-48aa-8057-338cc7a9d54c.png)<br>![image](https://user-images.githubusercontent.com/95737452/185780064-f144785f-9e89-4a07-9114-e7152a0271ed.png)
  * Dependencies: **First order** (That are specified at the creation of the package) and **Indirect** (Dependencies on which the first-order dependencies depend)
  * The meta information as specified within the **package.xml** file maybe customised as per requirement using various tags (maintainer, license, dependencies)

* ROS Nodes:
  * Node: An executable file within a ROS package. ROS nodes use a ROS client library (rospy/roscpp) to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.
  * roscore: The first thing you run when using ROS.
  * ![image](https://user-images.githubusercontent.com/95737452/185780664-2fc30fcf-e300-4a98-9068-d5227000c714.png)
  * rosnode list: To list out various nodes currently in execution.
  * rosnode info: To provide information about a certain node in execution.
  * rosrun: To execute a particular node.
* ROS Topics:
  * rostopic list: To list out various topics currently in excecution.
  * rostopic info: To provide information about a certain topic in use.  
  * rostopic Commands:<br>![image](https://user-images.githubusercontent.com/95737452/185781898-98cc828a-0b1a-4a7a-a243-a874c26d0dcd.png)<br> 
* rqt_graph: Creates a dynamic graph of what's going on in the system.<br>![image](https://user-images.githubusercontent.com/95737452/185781867-5b9b5a76-f428-4f90-a3dc-b15787affd30.png)
* rqt_plot: Gives the position of the turtle (in case of turtlesim) at a given instant over a span of time.

* ROS Messages: 
  * Communication on topics happens by sending ROS messages between nodes.
  * For the publisher and subscriber to communicate, the publisher and subscriber must send and receive the same type of message. This means that a topic type is defined by the message type published on it. The type of the message sent on a topic can be determined using rostopic type.<br>![image](https://user-images.githubusercontent.com/95737452/185782156-8644915f-5ae4-4763-83ea-0b8a9b21632e.png)<br>![image](https://user-images.githubusercontent.com/95737452/185782170-60d4528e-73ff-4624-a2e2-f7fab1ad3971.png)
  * rosmsg: Used to get the details of a message.<br>![image](https://user-images.githubusercontent.com/95737452/185782196-0090c1a5-8866-448e-bdd5-a5a3b892ffc1.png)
  * rostopic pub: Publishes data on to a topic currently advertised.<br>![image](https://user-images.githubusercontent.com/95737452/185782230-8bbb2e16-9c20-494a-953d-70649b67ffd3.png)










    
