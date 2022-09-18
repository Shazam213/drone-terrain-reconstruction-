# Core ROS Tutorials
## 1.1 Beginner level
<details>
<summary> 1) Installing and Configuring Your ROS Environment</summary>

* Installing ROS and setting ROS environment on your computer.
* A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:
        
        $printenv | grep ROS
* If they are not then you might need to 'source' some setup.*sh files. 
* rosbuild and catkin: methods for organizing and building your ROS code. catkin is recommended and used.
* If ROS Kinetic installed then you will have setup.*sh files in '/opt/ros/kinetic/', and you could source them like so: 

        $ source /opt/ros/kinetic/setup.bash

* You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc. This process allows you to install several ROS distributions (e.g. indigo and kinetic) on the same computer and switch between them. 

* Let's create and build a catkin workspace:

        $ mkdir -p ~/catkin_ws/src
        $ cd ~/catkin_ws/
        $ catkin_make

* The catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. 
* Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.*sh file:

        $ source devel/setup.bash

* To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.

        $ echo $ROS_PACKAGE_PATH/home/youruser/catkin_ws/src:/opt/ros/kinetic/share
</details>
<details>
<summary>2) Navigating the ROS Filesystem</summary>


* Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. Most users will want to use the prebuilt packages, but installing it from source is also quite simple. 
            
        sudo apt-get install ros-noetic-catkin
* Packages: Packages are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.
* Manifests (package.xml): A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc... 
* Code is spread across many ROS packages. Navigating with command-line tools such as ls and cd can be very tedious which is why ROS provides tools to help you. 
* rospack allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.
         
        $ rospack find [package_name]
* roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.
    
        $ roscd <package-or-stack>[/subdir]
* Note that roscd, like other ROS tools, will only find ROS packages that are within the directories listed in your ROS_PACKAGE_PATH. To see what is in your ROS_PACKAGE_PATH, type:

        $ echo $ROS_PACKAGE_PATH

* Your ROS_PACKAGE_PATH should contain a list of directories where you have ROS packages separated by colons. A typical ROS_PACKAGE_PATH might look like this:
    
        /opt/ros/kinetic/base/install/share
* roscd can also move to a subdirectory of a package or stack. 
* roscd log will take you to the folder where ROS stores log files. Note that if you have not run any ROS programs yet, this will yield an error saying that it does not yet exist.

        $ roscd log
* rosls allows you to ls directly in a package by name rather than by absolute path.

        $ rosls <package-or-stack>[/subdir]
  </details>
<details>
<summary> 3) Creating a ROS Package</summary>

* For a package to be considered a catkin package it must meet a few requirements: 
  
  * must contain a catkin compliant package.xml file
  * must contain a CMakeLists.txt which uses catkin.
  * must have its own folder 
* catkin_create_pkg script is used to create a new package in the a source location which has a src folder which contains its own Cmakelist
* to create a new package first we need to make the package and then we need to build the package.
* to make the package use:
        
        catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
    here the dependancies used generally are std_msgs, rospy and roscpp

    This will create a <package_name> folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave catkin_create_pkg. 
* the to build the package we have to come back to the source location and then use:
    
        $ cd ~/<source_location>
        $ catkin_make

    After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO_NAME.

    To add the workspace to your ROS environment you need to source the generated setup file:

    
        $ . ~/catkin_ws/devel/setup.bash
    after this the ros environment recongnises the new package and the new package can be used directly,
* there are various further ways to customize your package.<br>For more info [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
</details>
<details>
<summary>4) Understanding ROS Nodes</summary>

* Overview:
  * Nodes: A node is an executable that uses ROS to communicate with other nodes.
  * Messages: ROS data type used when subscribing or publishing to a topic.
  * Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
  * Master: Name service for ROS (i.e. helps nodes find each other)
  * rosout: ROS equivalent of stdout/stderr
  * roscore: Master + rosout + parameter server (parameter server will be introduced later) 
* roscore is the first thing you should run when using ROS.

        $ roscore
* rosnode displays information about the ROS nodes that are currently running. The rosnode list command lists these active nodes:

        $ rosnode list

* The rosnode info command returns information about a specific node.

        $ rosnode info /rosout
* rosrun allows you to use the package name to directly run a node within a package (without having to know the package path).

        $ rosrun [package_name] [node_name]
</details>
<details>
<summary>5) Understanding ROS Topics</summary>

* Two nodes communicate with each other using topics.
* One node **publishes** to a topic while the other node **subscribes** to the same topic thereby exchanging data in the form of messages.
* rqt_graph creates a dynamic graph of what's going on in the system. rqt_graph is part of the rqt package.<br>
    In a new terminal:

        $ rosrun rqt_graph rqt_graph
* The rostopic tool allows you to get information about ROS topics. <br>You can use the help option to get the available sub-commands for rostopic

        $ rostopic -h
* rostopic echo shows the data published on a topic.

        rostopic echo [topic]
* rostopic list returns a list of all topics currently subscribed to and published.

    Let's figure out what argument the list sub-command needs. In a new terminal run:

        $ rostopic list -h
* Communication on topics happens by sending ROS messages between nodes. For the publisher and subscriber to communicate, the publisher and subscriber must send and receive the same type of message. This means that a topic type is defined by the message type published on it. The type of the message sent on a topic can be determined using rostopic type. 
* rostopic type returns the message type of any topic being published.

        rostopic type [topic]
* rostopic pub publishes data on to a topic currently advertised.

        rostopic pub [topic] [msg_type] [args]
    eg:
        
        $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'

   * This is a pretty complicated example, so lets look at each argument in detail.

        * This command will publish messages to a given topic:

            rostopic pub

        * This option (dash-one) causes rostopic to only publish one message then exit:

            -1 

        * This is the name of the topic to publish to:

            /turtle1/cmd_vel

        * This is the message type to use when publishing to the topic:

            geometry_msgs/Twist

        * This option (double-dash) tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.

            --

      * We can publish a steady stream of commands using rostopic pub -r command:

            $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
* rostopic hz reports the rate at which data is published.

        rostopic hz [topic]
* rqt_plot displays a scrolling time plot of the data published on topics.

        $ rosrun rqt_plot rqt_plot
</details>
<details>
<summary>6) Understanding ROS Services and Parameters</summary>

* Services are another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response. 
* rosservice can easily attach to ROS's client/service framework with services. rosservice has many commands that can be used on services, as shown below:

        rosservice list         print information about active services
        rosservice call         call the service with the provided args
        rosservice type         print service type
        rosservice find         find services by service type
        rosservice uri          print service ROSRPC uri
* rosparam allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax. In simple cases, YAML looks very natural: 1 is an integer, 1.0 is a float, one is a string, true is a boolean, [1, 2, 3] is a list of integers, and {a: b, c: d} is a dictionary. rosparam has many commands that can be used on parameters, as shown below:

        rosparam set            set parameter
        rosparam get            get parameter
        rosparam load           load parameters from file
        rosparam dump           dump parameters to file
        rosparam delete         delete parameter
        rosparam list           list parameter names
</details>

<details>
<summary>7) Using rqt_console and roslaunch</summary>

* rqt_console attaches to ROS's logging framework to display output from nodes. rqt_logger_level allows us to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run. 
* roslaunch starts nodes as defined in a launch file. 
        
        $ roslaunch [package] [filename.launch]
  
</details>
<details>
<summary>8) Using rosed to edit files in ROS</summary>

* rosed is part of the rosbash suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package.

        $ rosed [package_name] [filename]
* The default editor for rosed is vim. The more beginner-friendly editor nano is included with the default Ubuntu install. You can use it by editing your ~/.bashrc file to include:

        export EDITOR='nano -w'
</details>
<details>
<summary>9) Creating a ROS msg and srv</summary>

* msg: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
* srv: an srv file describes a service. It is composed of two parts: a request and a response. 
* msg files are stored in the msg directory of a package, and srv files are stored in the srv directory.
* msgs are just simple text files with a field type and field name per line. The field types you can use are:

        int8, int16, int32, int64 (plus uint*)
        float32, float64
        string
        time, duration
        other msg files
        variable-length array[] and fixed-length array[C] 
* There is also a special type in ROS: Header, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have Header header. 
* srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. 
* rosmsg show command helps to view the msg created by you
        
        $ rosmsg show [message type]
* rossrv show command helps ROS to see the srv just created
        
        $ rossrv show <service type>
* for more info on creating msg and srv [click here](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
</details>
<details>
<summary>Review</summary>
Let's just list some of the commands we've used so far:

    rospack = ros+pack(age) : provides information related to ROS packages

    roscd = ros+cd : changes directory to a ROS package or stack

    rosls = ros+ls : lists files in a ROS package

    roscp = ros+cp : copies files from/to a ROS package
    rosmsg = ros+msg : provides information related to ROS message definitions
    rossrv = ros+srv : provides information related to ROS service definitions
    catkin_make : makes (compiles) a ROS package
        rosmake = ros+make : makes (compiles) a ROS package (if you're not using a catkin workspace) 
    catkin build: makes (compiles) a ROS package in an isolated manner while maintaining efficiency due to parallelisation
        catkin_make + catkin_make_isolated 
</details>
<details>
<summary>10) Writing a Simple Publisher and Subscriber (Python)</summary>

* To make any python file executable use:
        
        $ chmod +x <.py file>
* Writing a publisher node:<br>
  *  lets take an example of publisher node talker.py:
  
                1 #!/usr/bin/env python
                2 # license removed for brevity
                3 import rospy
                4 from std_msgs.msg import String
                5 
                6 def talker():
                7     pub = rospy.Publisher('chatter', String, queue_size=10)
                8     rospy.init_node('talker', anonymous=True)
                9     rate = rospy.Rate(10) # 10hz
                10     while not rospy.is_shutdown():
                11         hello_str = "hello world %s" % rospy.get_time()
                12         rospy.loginfo(hello_str)
                13         pub.publish(hello_str)
                14         rate.sleep()
                15 
                16 if __name__ == '__main__':
                17     try:
                18         talker()
                19     except rospy.ROSInterruptException:
                20         pass
  * Now, let's break the code down.


                1 #!/usr/bin/env python

        * Compulsory line, makes sure that script is executed as python script



                 3 import rospy
                 4 from std_msgs.msg import String

        * to write a ROS node we need to import rospy. 



                 7 pub = rospy.Publisher('chatter', String, queue_size=10)
                 8 rospy.init_node('talker', anonymous=True)
        * pub defines that your node is publishing to the topic named chatter, the message sent is of string type, and queue size limits the queued msgs to 10 if any subscriber is not receiving msgs.
        * init_node initializes the publisher node named 'talker' and gives it an anonymous value everytime it is run.

                 9 rate = rospy.Rate(10) # 10hz

        * helps looping the msgs at a desired rate. here 10 msgs are sent every second



                 10 while not rospy.is_shutdown():
                 11     hello_str = "hello world %s" % rospy.get_time()
                 12     rospy.loginfo(hello_str)
                 13     pub.publish(hello_str)
                 14     rate.sleep()

        * fairly standard rospy construct, checking the rospy.is_shutdown() flag and then doing work. "work" is a call to pub.publish(hello_str) that publishes a string to our chatter topic. The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.

        * This loop also calls rospy.loginfo(str), which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to rosout. rosout is a handy tool for debugging: you can pull up messages using rqt_console instead of having to find the console window with your Node's output.

        * std_msgs.msg.String is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that constructor args are in the same order as in the .msg file. You can also pass in no arguments and initialize the fields directly,


                17     try:
                18         talker()
                19     except rospy.ROSInterruptException:
                20         pass

        * In addition to the standard Python __main__ check, this catches a rospy.ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the sleep(). 
* Writing a Subscriber node:
  *   lets take an example of publisher node listener.py:
     
                1 #!/usr/bin/env python
                2 import rospy
                3 from std_msgs.msg import String
                4 
                5 def callback(data):
                6     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
                7     
                8 def listener():
                9 
                10     # In ROS, nodes are uniquely named. If two nodes with the same
                11     # name are launched, the previous one is kicked off. The
                12     # anonymous=True flag means that rospy will choose a unique
                13     # name for our 'listener' node so that multiple listeners can
                14     # run simultaneously.
                15     rospy.init_node('listener', anonymous=True)
                16 
                17     rospy.Subscriber("chatter", String, callback)
                18 
                19     # spin() simply keeps python from exiting until this node is stopped
                20     rospy.spin()
                21 
                22 if __name__ == '__main__':
                23     listener()
  * The code for listener.py is similar to talker.py, except we've introduced a new callback-based mechanism for subscribing to messages.



                15     rospy.init_node('listener', anonymous=True)
                16 
                17     rospy.Subscriber("chatter", String, callback)
                18 
                19     # spin() simply keeps python from exiting until this node is stopped
                20     rospy.spin()
        * init_node initializes a node named listener and gives it anonymous value everytime it is run.
        * rospy.Subscriber subscribes the node to the topic named chatter which is carrying msgs in String format. When new messages are received, callback is invoked with the message as the first argument. 
        * The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. 

</details>
<details>
<summary>11) Writing a Simple Service and Client</summary>

* To create a service and a client first you need to create a srv file which will contain a request and response for the service
* you then write the service and client nodes in the scripts directory
* There's very little to writing a service using rospy. We declare our node using init_node() and then declare our service:

        s = rospy.Service('<name_of_service>', <service_type>, callback function)
here the service type is always mentioned in the srv file.
* The client code for calling services is also simple. For clients you don't have to call init_node(). We first call:

        rospy.wait_for_service('<service_name>')
* Next we create a handle for calling the service:

        <handle_name> = rospy.ServiceProxy('<service_name>', <srv_file>)
* We can use this handle just like a normal function and call it in the client code later.
* for more info on creating service and client nodes [click here](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
</details>
<details>
<summary>12) Recording and playing back data</summary>

* the topic data collected is always saved in a bag file.
* The list of published topics are the only message types that could potentially be recorded in the data log file, as only published messages are recorded. 
* To record the published data. Open a new terminal window. In this window run the following commands:

                mkdir ~/bagfiles
                cd ~/bagfiles
                rosbag record -a

Here we are just making a temporary directory to record data and then running rosbag record with the option -a, indicating that all published topics should be accumulated in a bag file. 
* After some time of communication between the subscriber and publisher, you should see a file with a name that begins with the year, date, and time and the suffix .bag. This is the bag file that contains all topics published by any node in the time that rosbag record was running. 
* rosbag info command checks the contents of the bag file without playing it back. Execute the following command from the bagfiles directory:

        rosbag info <your bagfile>
* This tells us topic names and types as well as the number (count) of each message topic contained in the bag file.
* to replay the bag file run the following command in the directory where you took the original bag file:

        rosbag play <your bagfile>
* In its default mode rosbag play will wait for a certain period (.2 seconds) after advertising each message before it actually begins publishing the contents of the bag file. Waiting for some duration allows any subscriber of a message to be alerted that the message has been advertised and that messages may follow. If rosbag play publishes messages immediately upon advertising, subscribers may not receive the first several published messages. The waiting period can be specified with the -d option.
* To play the bag file from a particular time we can use -s argument. The -r argument, allows you to change the rate of publishing by a specified factor. If you execute:

        rosbag play -r 2 <your bagfile>
then the rate of publishing will be twice as that was present originally.
* when many topics are being published simultaneously we can only record a specific set of topics as our descrition using the -O argument is rosbag record.
* 
        rosbag record -O <your bagfile> <topic1> <topic2> ...
* rosbag is limited in its ability to exactly duplicate the behavior of a running system in terms of when messages are recorded and processed by rosbag record, and when messages are produced and processed when using rosbag play. 
</details>
<details>
<summary>13) Reading messages from a bag file</summary>

* for reading messages from a bag file [click here](http://wiki.ros.org/ROS/Tutorials/reading%20msgs%20from%20a%20bag%20file)
</details>
<details>
<summary>14) Getting started with roswtf</summary>

* roswtf examines your system to try and find problems. 
* you can go into any ROS package and try to find the problems in the package using roswtf

        $ roscd <package>
        $ roswtf
* roswtf will warn you about things that look suspicious but may be normal in your system. It can also report errors for problems that it knows are wrong. 
* If you find yourself stumped by a build or communication issue, try running it and seeing if it can point you in the right direction. 
</details>
<br>

## 1.2 Intermediate Level

<details>
<summary>1) Creating a ROS package by hand</summary>

* tool for creating ROS package is catkin_create_pkg
* catkin_create_pkg prevents mistakes and saves effort, but packages are just a directory and a simple XML file. 
* first we always have to create a manifest file. The package.xml file allows tools like rospack to determine information about what your package depends upon. 
* Then we need the CMakeLists.txt file so that catkin_make, which uses CMake for its more powerful flexibility when building across multiple platforms, builds the package. 
* That's all you need to start building a package in ROS using catkin. if you want it to actually start building something, you're going to need to learn a couple more CMake macros
</details>
<details>
<summary>2) Managing System dependencies</summary>

* ROS packages sometimes require external libraries and tools that must be provided by the operating system. These required libraries and tools are commonly referred to as system dependencies. In some cases these system dependencies are not installed by default. ROS provides a simple tool, rosdep, that is used to download and install system dependencies. 
        
        rosdep install [package]
if error occurs try:

        sudo rosdep init
        rosdep update
* While rosdep is the client tool, the reference is provided by rosdep rules, stored online in ros/rosdistro/rosdep on github.
* These rules are used when a dependency is listed that doesn't match the name of a ROS package built on the buildfarm. Then rosdep checks if there exists a rule to resolve it for the proper platform and package manager you are using. 
* When creating a new package, you might need to declare new system dependencies to the rosdep rules if they are not there yet. Just edit the file, add the dependency neededand send a pull request.

After that pull request has been merged, you need to run :

        $ rosdep update

and now that dependency will be resolved by rosdep.

You can test it with :

        $ rosdep resolve my_dependency_name3
</details>
<details>
<summary>3) Roslaunch tips for large projects</summary>

* for working with large projects [click here](http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects)
</details>