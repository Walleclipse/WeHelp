# WeHelp
## Abstract
There is a large population of wheelchair users. Most of the wheelchair users need help with daily tasks. However, according to recent reports, their needs are not properly satisfied due to the lack of caregivers. Therefore, in this project, we develop WeHelp, a shared autonomy system aimed for wheelchair users. A robot with WeHelp system has three modes, following mode, remote control mode and tele-operation mode. In the following mode, the robot follows the wheelchair user automatically via visual tracking. The wheelchair user can ask the robot follow them from behind, by the left or by the right. When the wheelchair user asks for help, the robot will recognize the command via speech recognition, and then switch to the tele-operation mode or remote control mode. In the tele-operation mode, the wheelchair user takes over the robot by a joy stick and control the robot to complete some complex tasks for their needs, such as opening doors, moving obstacles on the way, reaching objects on a high shelf or on the low ground, etc. In the remote control mode, a remote assistant takes over the robot and help the wheelchair user complete some complex tasks for their needs. Our evaluation shows that the pipeline is useful and practical for wheelchair users. Source code and demo of the paper is available at \url{https://github.com/Walleclipse/WeHelp}.

## Method

This work aims to develop a pipeline that can help wheelchair users with their daily life tasks. We implemented the WeHelp system on 
Stretch Research Edition robot (RE1) by [Hello Robot](https://hello-robot.com/product) - which is a novel mobile manipulator designed for domestic settings, as shown in \cref{fig:strecth}. Our WeHelp system is composed of five modules: speech recognition, visual tracking, mode switch-
ing, remote control interface, and teleoperation interface. The speech recognition module is used to recognize speech commands from wheelchair users. The visual tracking module served as a wheelchair follower on the behind or alongside the wheelchair user. Mode switching is used to change the following modes (following from behind, or following in accompany mode). In the remote-control function, a caregiver will take over the robot through the remote control interface. And in the teleoperation function, the wheelchair user takes over the robot themselves to finish their desired tasks. 

## Demo

Following from behind (robot’s view).  
  
<img src="demo/track_2.gif" width="400" height="300" alt="Following from behind (robot’s view)"/>  
  
Following on left (robot’s view). 
  
<img src="demo/track_3.gif" width="400" height="300" alt="Following on left (robot’s view)"/>  
  
Following from behind then switch to left mode. 
  
<img src="demo/track_1.gif" width="400" height="600" alt="Switch mode"/>  
  
