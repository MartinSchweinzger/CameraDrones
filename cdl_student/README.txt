This document describe some useful information of the code templates for Assignment 2, 3 and 4, here all in this "cdl_student" folder.

In this main folder, several folders are included:

1. "bags": rosbag files generated or to be used would be in this folder.
2. "external_libs": folder for thirdparty libraries, inside there is the 
                    apriltag-cpp library that is used in assignment 3/4.
3. "scripts": demo shell scripts for assignment 2 are placed. You may also
              want to write the scripts for A-3 and A-4 and put them here.
4. "src": folder for source codes required for assignment 2/3/4, where you 
          need to code your parts in. In there:
          
          "ethz": some packages needed for the assignments, you don't need
                  to do anything but to know the places they are being used.
                  
          "icg": packages that are highly-relevant to the assignments:
          
                 "cdl_common": some common packages, such as visualization tool.
                               the massege and service folders are also in this
                               place: "icg_msgs" and "icg_srvs". If you need to 
                               refer to messege and service, or create your new 
                               ones, do it here.
                               
                 "dla2_path_planner": template package for assignment 2 (A-2);
                 
                 "icg_apriltag_marker_detector": template package for assignment 3 (A-3);
                 
                 "task_scheduler_student": template package for assignment 4 (A-4);
                 
                 
To make them work in the virtual machine provided in A-1, put the folder "cdl_student" in the home directory, and treat it as a new workspace.

Prior to building the workspace, install octomap relevant pakcages:

sudo apt-get install ros-noetic-octomap
sudo apt-get install ros-noetic-octomap-rviz-plugins
sudo apt-get install ros-noetic-octomap-mapping
sudo apt-get install ros-noetic-dynamic-edt-3d

and the OMPL (https://ompl.kavrakilab.org/core/installation.html) library:

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-`rosversion -d`-ompl

Now start to build the workspace:

cd /home/student/cdl_student
catkin config --extend /opt/ros/noetic && catkin build -DCMAKE_BUILD_TYPE=Release -j4

Note-0: you may see there is an error regarding the "/ethz/glog_catkin" package, which I found it does not work with gcc/g++11.4 version.
        This issue can be avoided by switching to gcc/g++9.4 for the compilation, while any other possible solution could be used.
Note-1: you may also need to specify the directory of third party library (apriltag-cpp) to avoid errors regarding this.
Note-2: there will be many warnings (mostly version conflicts) when compiling the codes, just ignore them.

After successfully compiled, you can make it the default workspace by putting the following command to ~/.bashrc (remember to source it in the terminal to become valid):

source /home/student/cdl_student/devel/setup.bash

Now, you can start to work on the assignments.

Happy coding! :)

Jun Zhang (JZ)



