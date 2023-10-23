---
# multilingual page pair id, this must pair with translations of this page. (This name must be unique)
lng_pair: id_mcs
title: "Getting started with Mavros and PX4 sitl"

# post specific
# if not specified, .name will be used from _data/owner/[language].yml
author: L Shiva Rudra
# multiple category is not supported
category: Projects
# multiple tag entries are possible
tags: [ROS, python]
# thumbnail image for post
img: ":sitlpos/1.png"
# disable comments on this page
#comments_disable: true

# publish date
date: 2023-06-20 10:04:30 +0900

# seo
# if not specified, date will be used.
#meta_modify_date: 2023-01-09 10:04:30 +0900
# check the meta_common_description in _data/owner/[language].yml
#meta_description: ""

# optional
# please use the "image_viewer_on" below to enable image viewer for individual pages or posts (_posts/ or [language]/_posts folders).
# image viewer can be enabled or disabled for all posts using the "image_viewer_posts: true" setting in _data/conf/main.yml.
#image_viewer_on: true
# please use the "image_lazy_loader_on" below to enable image lazy loader for individual pages or posts (_posts/ or [language]/_posts folders).
# image lazy loader can be enabled or disabled for all posts using the "image_lazy_loader_posts: true" setting in _data/conf/main.yml.
#image_lazy_loader_on: true
# exclude from on site search
#on_site_search_exclude: true
# exclude from search engines
#search_engine_exclude: true
# to disable this page, simply set published: false or delete this file
#published: false
---

'SITL'(software in the loop) is used to test robotic system in simulation before real-world implementation.  In this blog, the following will be covered:
- Installing mavros, PX4-Autopilot and setting up the simulation environment.
- Making the drone move between the position waypoints using ROS and python programming.
- Wherever required the code has been commented to explain any formulas or concepts.

# Installations

## Installing dependencies:
Run the following commands in a terminal:
```bash
sudo apt install geographiclib-tools -y
sudo apt-get install ros-noetic-pluginlib
sudo apt-get install ros-noetic-tf2-ros
sudo apt-get install ros-noetic-tf
sudo apt-get install ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

## Installing Mavros:
- Creating a workspace:
```bash
cd ~/Documents && mkdir -p mavros_ws/src && cd mavros_ws/src
git clone https://github.com/B-Bhanu-Teja/mavros.git
```

Mavros can be installed from both source or directly from debian using binary install.
Here, the latter will be used for simplicity:
```bash
sudo apt install ros-{distro}-mavros ros-{distro}-mavros-extras
```

Replace {distro} with your ROS distribution(kinetic, melodic, noetic etc.)

Next, install GeographicLib dataset:
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

For installation from the source, follow this page: [Link](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

Install octomap:
```bash
sudo apt-get install ros-{distro}-octomap
```

Next step is to build the mavros workspace:
```bash
cd ~/Documents/mavros_ws
catkin build
```


---

## Installing QGroundControl:
Follow the steps in this page to download and install the QGroundControl app image:
[Link](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)

---
## Installing PX4-Autopilot:
```bash
cd ~/Documents && mkdir -p PX4-Autopilot/src && cd PX4-Autopilot/src
git clone -b v1.11.2 https://github.com/PX4/PX4-Autopilot.git
```

Here, the version v1.11.2 is used since it is tested and trustable
```bash
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```

The above step is used to build the sitl-gazebo. If it built without any errors, follow the next steps and launch the 'iris' in gazebo:
```bash
cd ~/Documents/PX4-Autopilot/src/PX4-Autopilot
source ~/Documents/mavros_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```

You will find the iris drone in the simulator as follows:




![Screenshot from 2023-10-21 01-24-09.png](:sitlpos/1.png)



- You should also be able to view the mavros messages using the 'rostopic list' command in the terminal. These topics are used to establish the communication from and to the flight controller using ROS framework. 
- PX4 uses MAVLink protocol to communicate with the groundstation(the system on which QGC is running here). Using mavros enables us to send or receive messages/services with ease. 

---

# Flying the Iris
After launching iris in gazebo follow the steps:
- Start the QGroundControl from terminal or directly from the app image(if required permissions are enabled as given in the [Link](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)) 
- In the QGC app click on takeoff button and the drone in gazebo will takeoff. 
- The flight mode now changes to 'Takeoff' and then to 'Hold' mode which can be viewed in QGC and also running the following in terminal:
```bash
rostopic echo /mavros/state
```




![Screenshot from 2023-10-21 01-22-54.png](:sitlpos/2.png)



This can also be established using python script. In fact, many more manual settings can be done by doing this. Follow the steps:
- Open a terminal and create a ROS pkg to put the python scripts:
```bash
cd ~/Documents && mkdir -p drone_ws/src && cd drone_ws/src
catkin_create_pkg test_drone std_msgs geometry_msgs nav_msgs mavros_msgs rospy
cd .. && catkin build
cd src/test_drone/src
touch offboard_sub.py
code .
```

Paste the following code in offboard_sub.py, the drone will go to offboard mode and takeoff to 1m height where it will hold:

```python
#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np

class fcuModes:#these fcu modes are to call services like takeoff, land, arm etc. provided by mavros. Flight modes like offboard can also be set.
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e
			def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e
		
	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s. Offboard Mode could not be set.")%e
			
class Controller:#this is the class which has the functionality to control the drone's movement for tasks like reaching the waypoints
    # initialization
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()

        self.STEP_SIZE = 2.0
        # Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        
        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 1.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
		
		def posCb(self, msg):#this acts like a position feedback from mavros to manipulate the error term  
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):#drone state msg given by mavros has information like drone flying mode(eg: offboard, hold etc.)
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.pose.position.x = self.local_pos.x
        self.sp.pose.position.y = self.local_pos.y
        self.sp.pose.position.z = self.local_pos.z

def main():
    rospy.init_node('setpoint_node', anonymous=True)
	modes = fcuModes()
	cnt = Controller()
	rate = rospy.Rate(50)
	rospy.Subscriber('mavros/state', State, cnt.stateCb)
	rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
	sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
	while (cnt.state.armed == 0):
        modes.setArm()
        rate.sleep()
        print("ARMING")
	cnt.sp.pose.position.x = 0
	cnt.sp.pose.position.y = 0
    cnt.sp.pose.position.z = 1.0
	
	cnt.sp.pose.orientation.x = 0
    cnt.sp.pose.orientation.y = 0
	cnt.sp.pose.orientation.z = 0
	cnt.sp.pose.orientation.w = -1
		
		modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    while not rospy.is_shutdown():
        sp_pub.publish(cnt.sp)    
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

---

# Giving Position waypoints 

Here, corners of a square of side 5m are given as position waypoints. Use the code provided below for both the cases: with and without yaw.

## Without Yaw:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e


class Controller:
    def __init__(self):
        self.drone_pose = PoseStamped()
        self.state = State()
        self.sp = PoseStamped()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        To convert euler angles to quaternion.
        
        Input params(in radians)
             roll: The rotation around x-axis.
             pitch: The rotation around y-axis.
             yaw: The rotation around z-axis.
        
        Output list
             [qx, qy, qz, qw]: The orientation in quaternion 
        """
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [self.qx, self.qy, self.qz, self.qw]

    def pose_cb(self,msg):
        self.drone_pose = msg

    def stateCb(self,msg):
        self.state = msg

def main():
    modes = fcuModes()
    cnt = Controller()
    rospy.init_node('position_wp_node',anonymous=True)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    sub_pose = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback=cnt.pose_cb)
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    tolerance_ = 0.1 #there will be a difference between position setpoint and actual position received by mavros. This is the tolerance for the error
    rate = rospy.Rate(20)

    while (cnt.state.armed == 0):
        modes.setArm()
        rate.sleep()
        print("ARMING")

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")
    
    waypoints_={1:[0.0,0.0,1.0],2:[5.0,0.0,1.0],3:[5.0,5.0,1.0],4:[0.0,5.0,1.0],5:[0.0,0.0,1.0]} #waypoints which drone should follow in a dict

    for i in range(1,6):
      cnt.sp.pose.position.x = waypoints_[i][0]
      cnt.sp.pose.position.y = waypoints_[i][1]
      cnt.sp.pose.position.z = waypoints_[i][2]
      drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
      sp = [cnt.sp.pose.position.x, cnt.sp.pose.position.y, cnt.sp.pose.position.z]
      while (np.linalg.norm(np.array(drone_pose) - np.array(sp))>tolerance_):
        drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
        sp_pub.publish(cnt.sp)
        rate.sleep()
    #here the euclidean distance between the position setpoint and actual position is measured upto a tolerance before going to next waypoint
		
    modes.setAutoLandMode()
    print("---------")
    print("LANDING")
    print("---------")
	#after reaching the final waypoint the drone lands 

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

The video for the implementation in Gazebo:
<iframe width="560" height="315" src="https://www.youtube.com/embed/XARA60t2K-Y?si=3Wy5mjnxHKwwmXsU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The rviz visualization of drone's odometry:


![Screenshot from 2023-10-23 01-01-41_.png](:sitlpos/3.png)

---

## With yaw:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import numpy as np


class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: ,%s")%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s")%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set.")%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set.")%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set.")%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set.")%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set.")%e


class Controller:
    def __init__(self):
        self.drone_pose = PoseStamped()
        self.state = State()
        self.sp = PoseStamped()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        To convert euler angles to quaternion.
        
        Input params(in radians)
             roll: The rotation around x-axis.
             pitch: The rotation around y-axis.
             yaw: The rotation around z-axis.
        
        Output list
             [qx, qy, qz, qw]: The orientation in quaternion 
        """
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [self.qx, self.qy, self.qz, self.qw]

    def pose_cb(self,msg):
        self.drone_pose = msg

    def stateCb(self,msg):
        self.state = msg

    def calc_yaw(self,curr_dir,init_dir):#yaw is estimated based on angle between direction vectors between waypoints. This is done with the help of cross-product of direction vectors
        vector_A = np.array(curr_dir)
        vector_B = np.array(init_dir)
        sin_theta = np.linalg.norm(np.cross(vector_A, vector_B)) / (np.linalg.norm(vector_A) * np.linalg.norm(vector_B))
        if sin_theta == 1:
            return np.pi/2.0 #if sin(theta)=1, then theta is pi/2
        else:
            return 0 #else if sin(theta)=0, then theta is 0. Since here the waypoints are corners of a square, only two values of yaw are possible: 0 and 90 degrees

def main():
    modes = fcuModes()
    cnt = Controller()
    rospy.init_node('position_wp_node',anonymous=True)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    sub_pose = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,callback=cnt.pose_cb)
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    tolerance_ = 0.1
    rate = rospy.Rate(20)

    while (cnt.state.armed == 0):
        modes.setArm()
        rate.sleep()
        print("ARMING")

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")
    
    waypoints_={1:[0.0,0.0,1.0],2:[5.0,0.0,1.0],3:[5.0,5.0,1.0],4:[0.0,5.0,1.0],5:[0.0,0.0,1.0]}
    init_dir = [0.0,0.0,1.0] #initial direction vector
    direction_vecs_ = {1:[0.0,0.0,1.0],2:[5.0,0.0,0.0],3:[0.0,5.0,0.0],4:[-5.0,0.0,0.0],5:[0.0,-5.0,0.0]} #dir. vectors are computed by element-wise subtraction of the adjacent waypoints
    yaw_=0.0
    yaw_old_ = 0.0
    for i in range(1,6):
      cnt.sp.pose.position.x = waypoints_[i][0]
      cnt.sp.pose.position.y = waypoints_[i][1]
      cnt.sp.pose.position.z = waypoints_[i][2]
      yaw_ = yaw_+cnt.calc_yaw(direction_vecs_[i],init_dir) #yaw is updated in world(here map) frame

      init_dir = direction_vecs_[i]
      cnt.sp.pose.orientation.x = cnt.get_quaternion_from_euler(0,0,yaw_old_)[0]
      cnt.sp.pose.orientation.y = cnt.get_quaternion_from_euler(0,0,yaw_old_)[1]
      cnt.sp.pose.orientation.z = cnt.get_quaternion_from_euler(0,0,yaw_old_)[2]
      cnt.sp.pose.orientation.w = cnt.get_quaternion_from_euler(0,0,yaw_old_)[3]
      drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
      sp = [cnt.sp.pose.position.x, cnt.sp.pose.position.y, cnt.sp.pose.position.z]
      yaw_old_ = yaw_ #newly computed yaw will be used at the next waypoint
      while (np.linalg.norm(np.array(drone_pose) - np.array(sp))>tolerance_):
        drone_pose = [cnt.drone_pose.pose.position.x, cnt.drone_pose.pose.position.y, cnt.drone_pose.pose.position.z]
        sp_pub.publish(cnt.sp)
        rate.sleep()
      
    
    modes.setAutoLandMode()
    print("---------")
    print("LANDING")
    print("---------")

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

The video for the implementation in Gazebo:
<iframe width="560" height="315" src="https://www.youtube.com/embed/nEfXL7jKJD8?si=MgRKrdb9oSeQGlZK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The rviz visualization of drone's odometry:

![Screenshot from 2023-10-23 01-16-32__.png](:sitlpos/4.png)


---