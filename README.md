# WeHelp
## Introduction
In this project, we develop WeHelp, a shared autonomy system aimed for wheelchair users. A robot with WeHelp system has three modes, following mode, remote control mode and tele-operation mode. In the following mode, the robot follows the wheelchair user automatically via visual tracking. The wheelchair user can ask the robot follow them from behind, by the left or by the right. When the wheelchair user asks for help, the robot will recognize the command via speech recognition, and then switch to the tele-operation mode or remote control mode. In the tele-operation mode, the wheelchair user takes over the robot by a joy stick and control the robot to complete some complex tasks for their needs, such as opening doors, moving obstacles on the way, reaching objects on a high shelf or on the low ground, etc. In the remote control mode, a remote assistant takes over the robot and help the wheelchair user complete some complex tasks for their needs. Our evaluation shows that the pipeline is useful and practical for wheelchair users.    
We implemented the WeHelp system on Stretch Research Edition robot (RE1) by [Hello Robot](https://hello-robot.com/product) - which is a novel mobile manipulator designed for domestic settings. 

<img src="demo/overview.png" width="500" height="200" alt="Illustration of the method pipeline."/>  

## Demo

Following from behind (robot’s view).  
  
<img src="demo/track_2.gif" width="200" height="150" alt="Following from behind (robot’s view)"/>  
  
Following on left (robot’s view). 
  
<img src="demo/track_3.gif" width="200" height="150" alt="Following on left (robot’s view)"/>  
  
Following from behind then switch to left mode. 
  
<img src="demo/track_1.gif" width="200" height="300" alt="Switch mode"/>  
  
