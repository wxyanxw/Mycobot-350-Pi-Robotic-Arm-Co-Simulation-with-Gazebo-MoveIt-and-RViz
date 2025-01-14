# mycobot_gazebo_moveit_rviz_simulation
Joint simulation of gazebo and rviz platforms on mycobot320 using moveit function package in ros1
This article is based on Ubuntu 20.04, Gazebo 11.11, and ROS Noetic.

---

### 1. **Explanation of the Co-Simulation Architecture**

Before starting the tutorial, I need to clarify the configuration process and the architecture of the co-simulation.

#### (1) Configuration Process:
We first need to download the MoveIt Assistant and the robotic arm model you want to simulate. The model can be in URDF or XACRO format. I will discuss the differences between the two in a later article. After a series of configurations with the MoveIt Assistant, it will generate a functional package. This package includes example code for single RViz simulation and Gazebo+RViz co-simulation. By running different launch files in this package, you can achieve different simulation routines, which is very convenient.

#### (2) Co-Simulation Architecture:
To be honest, I don’t fully understand the details of the co-simulation architecture, as I haven’t studied the underlying MoveIt source code. However, I can explain the general framework to some extent. In simple terms, there are four key components in co-simulation: the simulation environment (Gazebo), path planning visualization (RViz), path planning computation (MoveIt), and the robot controller (ROS Control). The simulation architecture roughly follows this process: we send control commands to the robot controller through RViz's control UI or a custom script that calls the MoveIt API. These commands can be either joint angles or target positions. If it’s the latter, MoveIt first performs path planning before sending the commands to the robot controller. RViz subscribes to the path planning topic to visualize the planned path. Once the robotic arm receives the control commands, it starts moving and publishes its motion information through a topic. Gazebo and RViz subscribe to this topic to visualize the actual motion.

My understanding of this co-simulation architecture is not deep, and there may be many mistakes. I welcome criticism and corrections from experts! However, the general framework should not deviate much.

---

### 2. **Downloading MoveIt**

Enter the following command in the terminal to download MoveIt. Replace the version with your ROS version:

```bash
sudo apt-get install ros-noetic-moveit
```

To verify if MoveIt is successfully installed, enter the following command:

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

If the following interface appears, the installation is successful. This MoveIt Assistant is an important tool we will use later to generate simulation routines:

[Insert image description here]

---

### 3. **Downloading the Mycobot 350 Pi Robotic Arm Model**

Official link: [https://github.com/elephantrobotics/mycobot_ros](https://github.com/elephantrobotics/mycobot_ros)

As far as I know, the official repository does not provide co-simulation routines (otherwise, it wouldn’t provide an unconfigured URDF). It only offers RViz path planning simulation. Therefore, we will only use the Mycobot 320 Pi URDF model file provided by the official repository.

After downloading, extract and open the folder:

[Insert image description here]

Copy the entire `mycobot_description` folder to your workspace `catkin_ws/src`:

[Insert image description here]

Compile it:

```bash
cd catkin_ws/
catkin_make
```

Open `mycobot_description/urdf` and find the model you want. I chose the Mycobot 320 Pi MoveIt version. Delete the other two URDF files:

[Insert image description here]

Open the URDF file and add the following code before the base link definition:

```xml
<link name="world"/>

<joint name="fixed_base" type="fixed">
  <parent link="world"/>
  <child link="base"/>
  <origin xyz="0 0 0.07" rpy="0 0 0"/>
</joint>
```

Like this:

[Insert image description here]

The purpose of adding this code is to fix the model in the Gazebo world. Without it, the model will collapse during co-simulation.

Next, scroll to the joint definition section at the end of the URDF file and set all velocity limits to 2.0:

[Insert image description here]

Why change the velocity limits? Because the official Mycobot model does not account for performance in Gazebo simulation, leading to issues like stuttering, inconsistent joint speeds, or even freezing. If you’re curious, you can try keeping the velocity limits at 0.

Finally, change the URDF file extension to `new_mycobot_pro_320_pi_moveit_2022.urdf.xacro`:

[Insert image description here]

Why make this change? Because we need to add a camera model later, and it’s not practical to modify the source code directly in the URDF. XACRO files allow us to include other XACRO files, similar to how programming languages work. Essentially, XACRO is an upgraded version of URDF. If you’re interested, you can search for the differences between URDF and XACRO. If you don’t change the extension now, you can still do it after generating the MoveIt configuration, but you’ll also need to modify the model’s launch file. I might write a separate article on converting URDF to XACRO and link it here.

Create a new folder, `mycobot_moveit_config`, to store the functional package generated by the MoveIt Assistant:

[Insert image description here]

---

### 4. **MoveIt Configuration**

Enter the following command to open the MoveIt Assistant:

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

Click "Create New MoveIt Configuration Package."

Click "Browse" on the right, locate the XACRO file we modified earlier, and click "Load Files."

If the following interface appears, the model has been successfully loaded:

[Insert image description here]

In the "Self-Collisions" tab, click "Generate." This automatically configures the robotic arm to ignore collisions between joints:

[Insert image description here]

Skip the "Virtual Joints" tab. This section is for configuring whether the robotic arm needs to be fixed to another mobile platform. Since we’ve already fixed it to the world, we don’t need to configure this.

The "Planning Groups" tab is crucial. Click "Add Group" and configure it as follows. `kdl_kinematics_plugin` is an inverse kinematics algorithm. Each algorithm has its advantages, so feel free to research them. `RRTConnect` is one of the trajectory planning methods. Again, you can research the differences. Click "Add Kin. Chain" (you can also include the entire robotic arm’s moving parts by joint or link):

[Insert image description here]

Select `world` as the base link and your end link as the tip link. I chose `link6`. Click "Save":

[Insert image description here]

The "Robot Poses" tab is for configuring the robot’s default poses. You can configure one or multiple poses. After configuration, you can use these predefined poses when controlling the robotic arm via RViz’s UI. Click "Add Pose." We’ll only set a "home" pose here. Click "Save":

[Insert image description here]

The "End Effectors" tab is for adding end effectors. We don’t need to add an end effector now, so I skipped this section. If needed, you can refer to other co-simulation blogs. Not configuring this won’t affect the robotic arm’s motion simulation.

The "Passive Joints" tab is for configuring passive joints, such as shells on the robotic arm’s links or additional components on the end effector. Since I don’t have these, I skipped this section.

The "Controllers" tab is important. Most co-simulation blogs online will tell you to select `position_controllers` and then modify configuration and launch files later. However, this is for older versions of MoveIt. I learned this the hard way. In the new version of MoveIt, you only need to click the "Auto Add" button:

[Insert image description here]

The MoveIt Assistant automatically selects a force controller for us. If you manually change it to another controller, the simulation will fail. Honestly, I’m not sure why this happens. If any experts can explain, I’d appreciate it. As for the differences between force, velocity, and position controllers, you can research them. The ROS Control official website has explanations, so I won’t go into detail here.

The "Simulation" tab is also important. The MoveIt Assistant automatically adds the necessary Gazebo plugin code to your XACRO file, such as weight, inertia coefficients, and virtual joint motors.

If the "Overwrite" button is grayed out before you click it, it might be because the model’s collision volumes overlap, causing issues with the generated model. In this case, you can try copying and pasting the code below into your XACRO file. However, since you didn’t click the "Overwrite" button, the MoveIt Assistant will generate a URDF file to replace your original XACRO file. The generated file will be located in `mycobot_moveit_config/config` and named `gazebo_firefighter.urdf`. Opening it will reveal that the code inside is what was generated in this tab. When you run the co-simulation, the model that actually launches will be this automatically generated file, not your original XACRO file. If you want to change the Gazebo launch path, open `mycobot_moveit_config/launch/gazebo.launch` and modify line 16, which defines the URDF launch path to the newly generated `gazebo_firefighter.urdf` file. Change it to:

```xml
<param name="robot_description" command="$(find xacro)/xacro '$(find mycobot_description)/urdf/mycobot_320_pi_2022/new_mycobot_pro_320_pi_moveit_2022.urdf.xacro'" />
```

This will ensure that the model launched is the original XACRO file.

If the "Overwrite" button is displayed normally, just click it. You don’t need to modify the above configuration. The image below shows the button grayed out because I already clicked it:

[Insert image description here]

Skip the "3D Perception" tab, as we’re not using it.

In the "Author Information" tab, fill in your details. MoveIt requires your contact information to generate the package.

Finally, the "Configuration Files" tab is for generating the functional package. Click "Browse," locate the `mycobot_moveit_config` folder we created earlier, click "Open," and then click "Generate" to complete the process.

---

### 5. **Installing Necessary Dependency Packages**

Install the ROS Control dependency package. Replace the version with your ROS version (mine is Noetic):

```bash
sudo apt-get install ros-noetic-ros-controllers
```

---

### 6. **Modifying PID**

Open `mycobot_moveit_config/config/ros_controllers.yaml` and set all P values to 1000:

[Insert image description here]

Why do this? Because with the default value of 100, the robotic arm’s force to reach the target position is too low, causing it to fail to reach the target point. This will result in warnings in the terminal and, when using a MoveIt API script to control the robotic arm to move to multiple points, the next action will terminate due to errors in the starting position caused by the previous action not reaching the target. Therefore, we increase the P value here.

---

### 7. **Launching the Co-Simulation**

Enter the following command to launch the co-simulation:

```bash
roslaunch mycobot_moveit_config demo_gazebo.launch
```

[Insert image description here]

Drag the trajectory ball in RViz, and the Gazebo model will move smoothly according to the trajectory.

At this point, the co-simulation is successful.

---

### 7. **Downloading the D435i Model and Gazebo Sensor Plugin Files for Gazebo Simulation**

Official link: [https://github.com/nilseuropa/realsense_ros_gazebo](https://github.com/nilseuropa/realsense_ros_gazebo)

The README on GitHub already provides a tutorial for inserting the camera:

[Insert image description here]

We will follow this template and adjust the position to add the camera.

---

### 8. **Testing the D435i Separately**

Copy the entire `realsense_ros_gazebo` package to `catkin_ws/src`.

Compile it:

```bash
catkin_make
```

Run the following command to test the camera:

```bash
roslaunch realsense_ros_gazebo simulation.launch
```

Open another terminal and enter the following command to check if the topics are being published correctly:

```bash
rostopic list
```

The result should look like this. On the left is the Gazebo simulation of the camera, and on the right is the topic list. You should see the RGB topic `/d435/color/image_raw` and other depth information topics like `/d435/depth/image_raw`:

[Insert image description here]

If these steps are completed successfully, proceed to the next step. If you encounter any errors, feel free to leave a comment or message me directly. I will do my best to help.

---

### 9. **Extracting the Model Files**

First, locate the URDF folder in your `mycobot_description` package where the robotic arm model is stored. Create a new folder named `camera` inside it (this is just for organization):

[Insert image description here]

Extract the downloaded `realsense_ros_gazebo-master` and open `realsense_ros_gazebo-master/xacro`. Copy the three files inside to the `camera` folder we just created. These files contain the Realsense camera model and Gazebo sensor configuration plugins. By including these files in the original XACRO model, you can directly use all the camera’s sensor functions.

Open `realsense_ros_gazebo-master/meshes/` and move the `realsense_d435.stl` file to the `camera` folder:

[Insert image description here]

Open the `depthcam` file. The first camera you see is the D435i:

[Insert image description here]

Scroll down to line 132 and change:

```xml
<horizontal_fov>${69.4*deg_to_rad}</horizontal_fov>
```

to:

```xml
<horizontal_fov>${30*deg_to_rad}</horizontal_fov>
```

Why make this change? The value `69.4` represents the field of view (FOV) angle. If you’ve played FPS games, you know that adjusting the FOV allows you to see more of the environment. The default FOV of `69.4` means the camera can see a 69.4-degree view. I’m not sure if the real D435i camera can achieve such a wide FOV, but changing it to `30` improves the realism of the simulation. You can try leaving it unchanged to see the difference.

Now look at line 4:

```xml
<xacro:include filename="$(find realsense_ros_gazebo)/xacro/inertia_calc.xacro"/>
```

Change it to:

```xml
<xacro:include filename="$(find mycobot_description)/urdf/mycobot_320_pi_2022/camera/inertia_calc.xacro"/>
```

This change is straightforward—it updates the path to the `inertia_calc.xacro` file to match our directory structure.

Similarly, look at line 40:

```xml
<mesh filename="package://realsense_ros_gazebo/meshes/realsense_d435.stl" />
```

Change it to:

```xml
<mesh filename="package://mycobot_description/urdf/mycobot_320_pi_2022/camera/realsense_d435.stl" />
```

This change updates the path to the STL file we moved to the `camera` folder.

Now look at line 248. The following section defines another Realsense R200 camera, which we don’t need. Delete the entire XACRO tag from line 248 to 448.

The modified XACRO file should look like this:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find mycobot_description)/urdf/mycobot_320_pi_2022/camera/inertia_calc.xacro"/>

  <!-- INTEL REALSENSE D435 -->

  <xacro:macro name="realsense_d435" params="sensor_name parent_link *origin rate">

    <xacro:property name="d435_cam_depth_to_left_ir_offset" value="0.0"/>
    <xacro:property name="d435_cam_depth_to_right_ir_offset" value="-0.050"/>
    <xacro:property name="d435_cam_depth_to_color_offset" value="0.015"/>
    <xacro:property name="d435_cam_width" value="0.090"/>
    <xacro:property name="d435_cam_height" value="0.025"/>
    <xacro:property name="d435_cam_depth" value="0.02505"/>
    <xacro:property name="d435_cam_mass" value="0.564"/>
    <xacro:property name="d435_cam_mount_from_center_offset" value="0.0149"/>
    <xacro:property name="d435_cam_depth_px" value="${d435_cam_mount_from_center_offset}"/>
    <xacro:property name="d435_cam_depth_py" value="0.0175"/>
    <xacro:property name="d435_cam_depth_pz" value="${d435_cam_height/2}"/>

    <!-- camera body, with origin at bottom screw mount -->
    <joint name="${sensor_name}_joint" type="fixed">
      <xacro:insert_block name="origin" />
      <parent link="${parent_link}"/>
      <child link="${sensor_name}_bottom_screw_frame" />
    </joint>
    <link name="${sensor_name}_bottom_screw_frame"/>

    <joint name="${sensor_name}_link_joint" type="fixed">
      <origin xyz="0 ${d435_cam_depth_py} ${d435_cam_depth_pz}" rpy="0 0 0"/>
      <parent link="${sensor_name}_bottom_screw_frame"/>
      <child link="${sensor_name}_link" />
    </joint>

    <link name="${sensor_name}_link">
      <visual>
        <origin xyz="${d435_cam_mount_from_center_offset} ${-d435_cam_depth_py} 0" rpy="${pi/2} 0 ${pi/2}"/>
        <geometry>
          <mesh filename="package://mycobot_description/urdf/mycobot_320_pi_2022/camera/realsense_d435.stl" />
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>
        <geometry>
          <box size="${d435_cam_depth} ${d435_cam_width} ${d435_cam_height}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${d435_cam_mass}" />
        <origin xyz="0 ${-d435_cam_depth_py} 0" rpy="0 0 0"/>
        <xacro:box_inertia m="${d435_cam_mass}" x="${d435_cam_depth}" y="${d435_cam_width}" z="${d435_cam_height}"/>
      </inertial>
    </link>

    <!-- camera depth joints and links -->
    <joint name="${sensor_name}_depth_joint" type="fixed">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <parent link="${sensor_name}_link"/>
      <child link="${sensor_name}_depth_frame" />
    </joint>
    <link name="${sensor_name}_depth_frame"/>

    <joint name="${sensor_name}_depth_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
      <parent link="${sensor_name}_depth_frame" />
      <child link="${sensor_name}_depth_optical_frame" />
    </joint>
    <link name="${sensor_name}_depth_optical_frame"/>

    <!-- camera left IR joints and links -->
    <joint name="${sensor_name}_left_ir_joint" type="fixed">
      <origin xyz="0 ${d435_cam_depth_to_left_ir_offset} 0" rpy="0 0 0" />
      <parent link="${sensor_name}_depth_frame" />
      <child link="${sensor_name}_left_ir_frame" />
    </joint>
    <link name="${sensor_name}_left_ir_frame"/>

    <joint name="${sensor_name}_left_ir_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
      <parent link="${sensor_name}_left_ir_frame" />
      <child link="${sensor_name}_left_ir_optical_frame" />
    </joint>
    <link name="${sensor_name}_left_ir_optical_frame"/>

    <!-- camera right IR joints and links -->
    <joint name="${sensor_name}_right_ir_joint" type="fixed">
      <origin xyz="0 ${d435_cam_depth_to_right_ir_offset} 0" rpy="0 0 0" />
      <parent link="${sensor_name}_depth_frame" />
      <child link="${sensor_name}_right_ir_frame" />
    </joint>
    <link name="${sensor_name}_right_ir_frame"/>

    <joint name="${sensor_name}_right_ir_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
      <parent link="${sensor_name}_right_ir_frame" />
      <child link="${sensor_name}_right_ir_optical_frame" />
    </joint>
    <link name="${sensor_name}_right_ir_optical_frame"/>

    <!-- camera color joints and links -->
    <joint name="${sensor_name}_color_joint" type="fixed">
      <origin xyz="0 ${d435_cam_depth_to_color_offset} 0" rpy="0 0 0" />
      <parent link="${sensor_name}_depth_frame" />
      <child link="${sensor_name}_color_frame" />
    </joint>
    <link name="${sensor_name}_color_frame"/>

    <joint name="${sensor_name}_color_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
      <parent link="${sensor_name}_color_frame" />
      <child link="${sensor_name}_color_optical_frame" />
    </joint>
    <link name="${sensor_name}_color_optical_frame"/>

    <!-- gazebo plugin -->

    <xacro:property name="deg_to_rad" value="0.01745329251994329577" />

    <gazebo reference="${sensor_name}_link">
      <self_collide>0</self_collide>
      <enable_wind>0</enable_wind>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
      <mu2>1</mu2>
      <fdir1>0 0 0</fdir1>
      <kp>1e+13</kp>
      <kd>1</kd>

      <sensor name="${sensor_name}_color" type="camera">
        <camera name="${sensor_name}">
          <horizontal_fov>${30*deg_to_rad}</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>RGB_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>1</visualize>
      </sensor>
      <sensor name="${sensor_name}_ired1" type="camera">
        <camera name="${sensor_name}">
          <horizontal_fov>${85.2*deg_to_rad}</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>90</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="${sensor_name}_ired2" type="camera">
        <camera name="${sensor_name}">
          <horizontal_fov>${85.2*deg_to_rad}</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>L_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>90</update_rate>
        <visualize>0</visualize>
      </sensor>
      <sensor name="${sensor_name}_depth" type="depth">
        <camera name="${sensor_name}">
          <horizontal_fov>${85.2*deg_to_rad}</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.100</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>90</update_rate>
        <visualize>0</visualize>
      </sensor>
    </gazebo>

    <gazebo>
      <plugin name="${sensor_name}" filename="librealsense_gazebo_plugin.so">
        <prefix>${sensor_name}_</prefix>
        <depthUpdateRate>${rate}</depthUpdateRate>
        <colorUpdateRate>${rate}</colorUpdateRate>
        <infraredUpdateRate>${rate}</infraredUpdateRate>
        <depthTopicName>depth/image_raw</depthTopicName>
        <depthCameraInfoTopicName>depth/camera_info</depthCameraInfoTopicName>
        <colorTopicName>color/image_raw</colorTopicName>
        <colorCameraInfoTopicName>color/camera_info</colorCameraInfoTopicName>
        <infrared1TopicName>infra1/image_raw</infrared1TopicName>
        <infrared1CameraInfoTopicName>infra1/camera_info</infrared1CameraInfoTopicName>
        <infrared2TopicName>infra2/image_raw</infrared2TopicName>
        <infrared2CameraInfoTopicName>infra2/camera_info</infrared2CameraInfoTopicName>
        <colorOpticalframeName>${sensor_name}_color_optical_frame</colorOpticalframeName>
        <depthOpticalframeName>${sensor_name}_depth_optical_frame</depthOpticalframeName>
        <infrared1OpticalframeName>${sensor_name}_infrared1_optical_frame</infrared1OpticalframeName>
        <infrared2OpticalframeName>${sensor_name}_infrared2_optical_frame</infrared2OpticalframeName>
        <rangeMinDepth>0.2</rangeMinDepth>
        <rangeMaxDepth>10.0</rangeMaxDepth>
        <pointCloud>false</pointCloud>
        <pointCloudTopicName>depth/color/points</pointCloudTopicName>
        <pointCloudCutoff>0.25</pointCloudCutoff>
        <pointCloudCutoffMax>9.0</pointCloudCutoffMax>
      </plugin>
    </gazebo>

  </xacro:macro>

</robot>
```

Make sure to modify the paths according to your directory structure!

---

### 10. **Adding the Camera to the Robotic Arm XACRO File**

Open the robotic arm’s XACRO model file `new_mycobot_pro_320_pi_moveit_2022.urdf.xacro`. At line 145, add the following code:

```xml
<xacro:include filename="$(find mycobot_description)/urdf/mycobot_320_pi_2022/camera/depthcam.xacro"/>

<xacro:realsense_d435 sensor_name="d435" parent_link="link6" rate="10">
  <origin rpy="0 -1.5708 1.5708 " xyz="0 -0.05 -0.015"/>
</xacro:realsense_d435>
```

This code includes the D435i model file and sets its parent link to `link6`. It also configures its position. You can adjust the position if needed.

Now, run the launch command to test the camera placement and topic publishing:

```bash
roslaunch mycobot_moveit_config demo_gazebo.launch
```

The result:

[Insert image description here]

Check the topic publishing status:

```bash
rostopic list
```

You should see the `/d435/color/image_raw` topic, indicating that the camera configuration is successful.

---

### 11. **Subscribing to the RGB Topic in Python and Performing Visual Recognition**

At this point, the Gazebo simulation environment is successfully set up. Next, we will process the published simulation information. We will write a Python script to read the `/d435/color/image_raw` topic and display it as a virtual camera. Then, we will use OpenCV visual recognition algorithms to calculate the center coordinates of a red block.

Ubuntu comes with Python pre-installed, so we don’t need to install it separately. However, we need to install the `python-is-python3` dependency to allow us to use the `python` command instead of `python3`, as well as some OpenCV dependencies:

```bash
sudo apt-get install python-is-python3
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install python3-opencv
```

In the `catkin/src` directory, enter the following command to create a Python package:

```bash
catkin_create_pkg mycobot_moveit_py_image_sub cv_bridge rospy sensor_msgs
```

This package includes three dependencies: `cv_bridge`, `rospy`, and `sensor_msgs`, which are the OpenCV library, Python compilation library, and Gazebo sensor information type library, respectively.

Enter the following command to create a `script` folder to store Python files:

```bash
mkdir script
```

Navigate to the `script` folder and create a Python file named `mycobot_moveit_py_image_sub.py`:

```bash
touch mycobot_moveit_py_image_sub.py
```

Copy the following source code:

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque

# Set red threshold in HSV space
redLower = np.array([0, 80, 50])
redUpper = np.array([8, 255, 220])
mybuffer = 64  # Initialize the list of tracked points
pts = deque(maxlen=mybuffer)

# Create a publisher
pub = rospy.Publisher('/red_object_center', PointStamped, queue_size=10)

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Get image dimensions
    height, width, _ = cv_image.shape
    center_x = width // 2
    center_y = height // 2

    # Convert to HSV space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Build a mask based on the threshold
    mask = cv2.inRange(hsv, redLower, redUpper)
    # Erode the mask
    mask = cv2.erode(mask, None, iterations=2)
    # Dilate the mask to remove noise
    mask = cv2.dilate(mask, None, iterations=2)
    # Detect contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None  # Initialize the centroid of the largest contour

    # If contours exist
    if len(cnts) > 0:
        # Find the largest contour
        c = max(cnts, key=cv2.contourArea)
        # Determine the enclosing circle of the largest contour
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # Calculate the moments of the contour
        M = cv2.moments(c)
        # Calculate the centroid
        center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
        cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        
        # Publish the coordinate information
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "camera_frame"  # Assume the camera frame is named "camera_frame"
        point_msg.point.x = (-center[0] + 319)*0.0002  # Convert pixel coordinates to Gazebo coordinates
        point_msg.point.y = 0
        point_msg.point.z = (-center[1] + 239)*0.0002  # Convert pixel coordinates to Gazebo coordinates
        pub.publish(point_msg)

    # Display the image
    cv2.imshow("RGB Image", cv_image)
    cv2.waitKey(30)

def main():
    rospy.init_node('rgb_image_subscriber', anonymous=True)
    
    # Subscribe to the RGB image topic
    rospy.Subscriber("/d435/color/image_raw", Image, image_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
```

A brief explanation of the key parts of the code:
- This line subscribes to the `/d435/color/image_raw` topic, receives the image message, and executes the `image_callback` callback function:

```python
rospy.Subscriber("/d435/color/image_raw", Image, image_callback)
```

- The callback function converts the image message, performs visual recognition, displays the image, and publishes the recognition result to the `/red_object_center` topic.
- Create a subscriber and publish the results:

```python
pub = rospy.Publisher('/red_object_center', PointStamped, queue_size=10)
...
pub.publish(point_msg)
```

If you’re interested in the source code, you can copy it into ChatGPT or another AI platform for further analysis. This is a great way to learn quickly. I won’t go into more detail here.

I recommend a useful domestic large model: [https://www.deepseek.com/](https://www.deepseek.com/)

Save the code and exit. Return to the catkin workspace and compile:

```bash
catkin_make
```

After successful compilation, you can run the program. Launch the simulation:

```bash
roslaunch mycobot_moveit_config demo_gazebo.launch
```

Open another terminal and run:

```bash
rosrun mycobot_moveit_py_image_sub mycobot_moveit_py_image_sub.py
```

Place any red object in front of the camera:

[Insert image description here]

Check the `/red_object_center` topic to see if the desired results are being published:

```bash
rostopic echo /red_object_center
```

The output should show the offset distance between the target point and the center:

[Insert image description here]

Why are the axes X and Z? Because in the robotic arm’s end-effector coordinate system, up and down are Z, left and right are X, and forward and backward are Y. The code has already converted the axes and coordinates, replacing the original Y with Z and shifting the origin from the bottom-left corner to the center. The coordinate values are multiplied by 0.0002 to convert them into Gazebo coordinates.

At this point, the camera has been successfully added to the model, and the camera image data has been read and processed for visual recognition! If you have any questions, feel free to leave a comment, and I’ll do my best to answer!

---

### 12. **Creating the Package**

Enter the following command to create the package and add the necessary dependencies:

```bash
catkin_create_pkg mycobot_moveit_py_ctrl geometry_msgs moveit_commander moveit_msgs rospy std_msgs
```

Navigate to the package and create a `script` folder:

```bash
mkdir script
```

Navigate to the `script` folder and create the Python files `mycobot_moveit_py_search.py` and `mycobot_moveit_py_move.py`:

```bash
touch mycobot_moveit_py_search.py
touch mycobot_moveit_py_move.py
```

After creating the files, right-click on them, go to the permissions tab, and allow them to be executed as programs.

---

### 13. **Writing the Code**

Next, we’ll write the Python script code. I’ll provide the code directly, with explanations in the comments.

#### `mycobot_moveit_py_search.py`:

```python
#!/usr/bin/env python

import rospy  # Import the ROS Python client library
import moveit_commander  # Import the MoveIt! Python interface
import geometry_msgs.msg  # Import the geometry message types
from sensor_msgs.msg import JointState  # Import the joint state message type
from moveit_commander.conversions import pose_to_list  # Import the pose conversion function
import subprocess  # Import the subprocess management module

joint_velocities = None  # Initialize the joint velocity variable
failure_count = 0  # Initialize the failure counter

def joint_states_callback(msg):
    global joint_velocities
    joint_velocities = msg.velocity  # Update the global variable joint_velocities with the received joint velocities

def move_arm_increment(move_group, x_increment, y_increment, z_increment):
    global failure_count
    end_effector_link = move_group.get_end_effector_link()  # Get the end-effector link name
    current_pose = move_group.get_current_pose(end_effector_link).pose  # Get the current end-effector pose

    # Calculate the new target pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x + x_increment
    target_pose.position.y = current_pose.position.y + y_increment
    target_pose.position.z = current_pose.position.z + z_increment

    # Maintain the current orientation
    target_pose.orientation = current_pose.orientation

    move_group.set_start_state_to_current_state()  # Set the start state to the current state
    move_group.set_pose_target(target_pose)  # Set the target pose

    while True:
        plan = move_group.plan()  # Plan the path
        if plan[0]:
            rospy.loginfo("Plan (pose goal) succeeded")
            move_group.execute(plan[1], wait=True)  # Execute the planned path
            failure_count = 0  # Reset the failure counter
            break
        else:
            rospy.loginfo("Plan (pose goal) failed")
            failure_count += 1
            if failure_count >= 3:
                rospy.loginfo("Planning failed 3 times, returning to home position and restarting the program.")
                move_group.set_named_target("home")  # Set the target to the "home" position
                move_group.go(wait=True)  # Move to the "home" position
                subprocess.call(["rosrun", "mycobot_moveit_py_crtl", "mycobot_moveit_py_search.py"])  # Restart the program
                rospy.signal_shutdown("Restarting the program")  # Shut down the node
                return
        
    while True:
        current_pose = move_group.get_current_pose(end_effector_link).pose  # Get the current end-effector pose
        if (abs(current_pose.position.x - target_pose.position.x) <= 0.01 and
            abs(current_pose.position.y - target_pose.position.y) <= 0.01 and
            abs(current_pose.position.z - target_pose.position.z) <= 0.01):
            break
        else:
            rospy.sleep(0.5)  # Wait for 0.5 seconds

    rospy.sleep(1)  # Wait for 1 second

    # Check if the joints are static
    while True:
        if joint_velocities is not None:
            if all(abs(vel) < 0.003 for vel in joint_velocities):
                rospy.loginfo("All joints are static")
                break
        rospy.sleep(0.5)  # Wait for 0.5 seconds

def main():
    moveit_commander.roscpp_initialize([])  # Initialize moveit_commander
    rospy.init_node('control_node', anonymous=True)  # Initialize the ROS node

    group_name = "arm"  # Set the robotic arm group name
    move_group = moveit_commander.MoveGroupCommander(group_name)  # Create a MoveGroupCommander object

    reference_frame = "world"  # Set the reference frame
    move_group.set_pose_reference_frame(reference_frame)

    move_group.allow_replanning(True)  # Allow replanning
    move_group.set_goal_position_tolerance(0.001)  # Set the position tolerance
    move_group.set_goal_orientation_tolerance(0.01)  # Set the orientation tolerance
    move_group.set_max_acceleration_scaling_factor(0.1)  # Set the maximum acceleration scaling factor
    move_group.set_max_velocity_scaling_factor(0.1)  # Set the maximum velocity scaling factor
    move_group.set_planning_time(5.0)  # Set the planning time to 5 seconds
    move_group.set_num_planning_attempts(10)  # Set the number of planning attempts to 10

    # Subscribe to the /joint_states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    move_group.set_named_target("home")  # Set the target to the "home" position
    move_group.go(wait=True)  # Move to the "home" position
    
    rospy.sleep(1)  # Wait for 1 second

    while not rospy.is_shutdown():
        x_increment = 0  # Example increment
        y_increment = 0  # Example increment
        z_increment = -0.03  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0.05  # Example increment
        y_increment = 0  # Example increment
        z_increment = 0  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # Example increment
        y_increment = 0  # Example increment
        z_increment = -0.12  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = -0.1  # Example increment
        y_increment = 0  # Example increment
        z_increment = 0  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # Example increment
        y_increment = 0  # Example increment
        z_increment = 0.12  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0.05  # Example increment
        y_increment = 0  # Example increment
        z_increment = 0  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        x_increment = 0  # Example increment
        y_increment = 0  # Example increment
        z_increment = 0.03  # Example increment

        move_arm_increment(move_group, x_increment, y_increment, z_increment)

        move_group.set_named_target("home")  # Set the target to the "home" position
        move_group.go(wait=True)  # Move to the "home" position
    
    moveit_commander.roscpp_shutdown()  # Shut down moveit_commander

if __name__ == '__main__':
    main()
```

The overall execution flow of the code:
1. Import necessary ROS and MoveIt! libraries.
2. Initialize global variables `joint_velocities` and `failure_count`.
3. Define the `joint_states_callback` function to update joint velocities.
4. Define the `move_arm_increment` function to incrementally move the robotic arm and reset or restart the program if planning fails.
5. In the `main` function:
   - Initialize `moveit_commander` and the ROS node.
   - Set the robotic arm group and reference frame.
   - Configure `MoveGroupCommander` parameters.
   - Subscribe to the `/joint_states` topic to get joint velocities.
   - Move the robotic arm to the "home" position.
   - Enter a loop to incrementally move the robotic arm and return to the "home" position after each move.
   - Shut down `moveit_commander` when the program ends.

An important step is to add error detection between the target point and the current point, as well as joint static detection, after each planning step. This ensures that the starting pose for the next planning step is the same as the current pose. This is reflected at the end of the `move_arm_increment` function.

#### `mycobot_moveit_py_move.py`:

```python
#!/usr/bin/env python

import rospy  # Import the ROS Python library
from geometry_msgs.msg import PointStamped  # Import the PointStamped message type
import time  # Import the time library
import subprocess  # Import the subprocess library
import signal  # Import the signal library
import os  # Import the operating system library
import moveit_commander  # Import the MoveIt Commander library
import geometry_msgs.msg  # Import the geometry_msgs message library
from sensor_msgs.msg import JointState  # Import the JointState message type

# Global variables
joint_velocities = None  # Global variable to store joint velocities
failure_count = 0  # Global variable to store failure count

# Function to terminate the mycobot_moveit_py_search.py process
def terminate_process():
    try:
        # Find the process ID of mycobot_moveit_py_search.py
        output = subprocess.check_output(["pgrep", "-f", "mycobot_moveit_py_search.py"])
        pids = output.decode().strip().split('\n')
        for pid in pids:
            if pid:
                os.kill(int(pid), signal.SIGTERM)  # Terminate the process
                print(f"Terminated process with PID {pid}")
    except subprocess.CalledProcessError:
        print("No process found with the name mycobot_moveit_py_search.py")

# Custom function to handle received xyz increment messages
def custom_function(msg, move_group):
    x_increment = msg.point.x  # Extract x increment from the message
    y_increment = msg.point.y  # Extract y increment from the message
    z_increment = msg.point.z  # Extract z increment from the message
    
    rospy.loginfo(f"Received xyz increments: x={x_increment}, y={y_increment}, z={z_increment}")  # Print xyz increment information
    
    move_arm_increment(move_group, x_increment, y_increment, z_increment)  # Call the move_arm_increment function
    
    if abs(x_increment) <= 0.01 and abs(y_increment) <= 0.01 and abs(z_increment) <= 0.01:
        print("Finish!, exiting...")
        rospy.signal_shutdown("Condition met")  # Shut down the node if the condition is met

# Main listener function
def listener():
    rospy.init_node('terminate_node', anonymous=True)  # Initialize the ROS node
    
    moveit_commander.roscpp_initialize([])  # Initialize moveit_commander
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)  # Create a MoveGroupCommander object
    
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)  # Subscribe to the /joint_states topic
    
    while not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message('/red_object_center', PointStamped, timeout=None)  # Wait for a message on the /red_object_center topic
            terminate_process()  # Terminate the mycobot_moveit_py_search.py process
            while True:
                if joint_velocities is not None:
                    if all(abs(vel) < 0.003 for vel in joint_velocities):  # Check if all joints are static
                        rospy.loginfo("All joints are static")
                        break
            rospy.sleep(1)            
            custom_function(msg, move_group)  # Execute the custom function
        except rospy.ROSInterruptException:
            break

# Joint state callback function
def joint_states_callback(msg):
    global joint_velocities
    joint_velocities = msg.velocity  # Update the global variable joint_velocities

# Function to move the robotic arm
def move_arm_increment(move_group, x_increment, y_increment, z_increment):
    global failure_count
    end_effector_link = move_group.get_end_effector_link()  # Get the end-effector link
    current_pose = move_group.get_current_pose(end_effector_link).pose  # Get the current pose

    target_pose = geometry_msgs.msg.Pose()  # Create a target pose object
    target_pose.position.x = current_pose.position.x + x_increment  # Calculate the new x target position
    target_pose.position.y = current_pose.position.y + y_increment  # Calculate the new y target position
    target_pose.position.z = current_pose.position.z + z_increment  # Calculate the new z target position

    target_pose.orientation = current_pose.orientation  # Maintain the current orientation

    move_group.set_start_state_to_current_state()  # Set the start state to the current state
    move_group.set_pose_target(target_pose)  # Set the target pose

    while True:
        plan = move_group.plan()  # Plan the path
        if plan[0]:
            rospy.loginfo("Plan (pose goal) succeeded")
            move_group.execute(plan[1], wait=True)  # Execute the path
            failure_count = 0  # Reset the failure counter
            break
        else:
            rospy.loginfo("Plan (pose goal) failed")
            failure_count += 1
            if failure_count >= 3:
                rospy.loginfo("Planning failed 3 times, returning to home position and restarting the program.")
                move_group.set_named_target("home")  # Set the target position to home
                move_group.go(wait=True)  # Move to the home position
                subprocess.call(["rosrun", "mycobot_moveit_py_crtl", "mycobot_moveit_py_search.py"])  # Restart the program
                rospy.signal_shutdown("Restarting the program")  # Shut down the node
                return
        
    while True:
        current_pose = move_group.get_current_pose(end_effector_link).pose  # Get the current pose
        if (abs(current_pose.position.x - target_pose.position.x) <= 0.01 and
            abs(current_pose.position.y - target_pose.position.y) <= 0.01 and
            abs(current_pose.position.z - target_pose.position.z) <= 0.01):  # Check if the target position is reached
            break
        else:
            rospy.sleep(0.5)

    rospy.sleep(1)

    while True:
        if joint_velocities is not None:
            if all(abs(vel) < 0.003 for vel in joint_velocities):  # Check if all joints are static
                rospy.loginfo("All joints are static")
                break
        rospy.sleep(0.5)

if __name__ == '__main__':
    listener()  # Start the main listener function
```

The code execution flow and functionality:
1. Initialize the ROS node: Create a ROS node named `terminate_node`.
2. Initialize MoveIt: Use `moveit_commander` to initialize MoveIt and create a `MoveGroupCommander` object to control the robotic arm.
3. Subscribe to joint states: Subscribe to the `/joint_states` topic to get joint velocity information.
4. Listen for messages: Wait for a message on the `/red_object_center` topic, which contains xyz increment information.
5. Terminate the process: Upon receiving the message, terminate the `mycobot_moveit_py_search.py` process.
6. Check joint static status: Ensure all joints are static to stabilize the robotic arm.
7. Execute the custom function: Call the `custom_function` to handle the received xyz increment message.
8. Move the robotic arm: Calculate the new target pose based on the xyz increments, plan and execute the path.
9. Check joint static status: Ensure all joints are static again to stabilize the robotic arm.
10. Condition check: If the xyz increments are very small, shut down the node.

After copying the code, save and exit. Return to the workspace and compile:

```bash
catkin_make
```

---

### 14. **Adding a Gray Wall and Red Button to the Model**

Navigate to the root directory of the robotic arm XACRO model and create a `wall.xacro` file:

```bash
touch wall.xacro
```

Add the model code for the wall and red button:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="wall">
    <visual>
      <origin xyz="0 -0.4 0.5" rpy="0 0 0" />
      <geometry >
	<box size="1 0.01 1" />
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
      </material>
    </visual> 
    <collision>
      <origin xyz="0 -0.4 0.5" rpy="0 0 0" />
      <geometry >
        <box size="1 0.01 1" />
      </geometry>
    </collision>      
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial> 
  </link>
  
  <joint name="wall_pos" type="fixed">
      <parent link="base"/>
      <child link="wall"/>
  </joint>
  
 
  <link name="buttom">
    <visual>
      <origin xyz="-0.09 -0.39 0.5" rpy="0 0 0" />
      <geometry >
	<box size="0.05 0.01 0.05" />
      </geometry>

    </visual> 
    <collision>
      <origin xyz="-0.09 -0.39 0.5" rpy="0 0 0" />
      <geometry >
        <box size="0.05 0.01 0.05" />
      </geometry>
    </collision>      
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial> 
  </link>
  
  <joint name="buttom_pos" type="fixed">
      <parent link="wall"/>
      <child link="buttom"/>
  </joint>

    
  <gazebo reference="wall">
       <material>Gazebo/LightGray</material>
  </gazebo>

  <gazebo reference="buttom">
       <material>Gazebo/Red</material>
  </gazebo>

</robot>
```

Open the model file and include the `wall.xacro` file. Add the following code at the beginning of the `robot` tag:

```xml
<xacro:include filename="$(find mycobot_description)/urdf/mycobot_320_pi_2022/wall.xacro"/>
```

Save and exit.

---

### 15. **Executing the Program**

Run four programs: co-simulation -> visual recognition -> move to target -> loop scanning.

Enter the following commands:

```bash
roslaunch mycobot_moveit_config demo_gazebo.launch
rosrun mycobot_moveit_py_image_sub mycobot_moveit_py_image_sub.py
rosrun mycobot_moveit_py_ctrl mycobot_moveit_py_move.py
rosrun mycobot_moveit_py_ctrl mycobot_moveit_py_search.py
```

The final running scene:

[Insert image description here]

The camera accurately lands on the center of the red block, and the simulation is successful.

---

### 16. **Summary**

The most challenging part of this small project was configuring the co-simulation environment, as described in my first article: "1. Gazebo+MoveIt+RViz Co-Simulation for the Latest MoveIt Version of the Mycobot 350 Pi Robotic Arm." The subsequent motion scripts, visual recognition, and their integration were not as difficult to write. The main challenge was understanding the code logic, which isn’t too complex. However, you need to refer to the official API examples and AI-generated code explanations frequently. Therefore, I didn’t explain the code line by line, as these explanations are available online.

With that, the explanation of this small project comes to an end. If you have any questions or notice any mistakes, please let me know in the comments. Thank you!
