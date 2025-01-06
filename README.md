# ROS2 Project

> ⚠️ **Attention**  
> Before making any changes in the project, make sure to be on the page *ie* to have the last version of the project : **git pull**.  
Check the **last section** to learn how to do it !

## Structure

### Structure of the document : 

* First part : Useful commands
* Second part : Github guide

### Structure of the project

* All the ROS2 packages are located in the **commande** folder  
* In each ROS2 packages is located a README file to explain how to start the nodes inside the package
* The used sdf file, **diff_drive.sdf**, is located in the **world** folder
* The used launch file, **MCAR.launch.py**, is located in the **launch** folder

Here find a detail of the ROS2 packages in the project :


#### Package *turtleFollower*

* **centered_cmd** : makes two turtles following a middle point between them contained in a trajectory
* **Follow.py** : makes a turtle following a leader turtle by using PID controllers in turtleSim
* **Follow_etat.py** : makes two turtles following each other by using a state equation (gains to adjust) in turtleSim
* **node_cmdTurtle1.py** : bases for creating a subcriber, a publisher and how to use them for moving turtles in turtleSim  
* **node_follow_trajectory.py** : generation of a random trajectory which is followed by a turtle
* **utils** : regroup all the handmade functions needed in the package
* **README** : explain how to start the nodes of the package

#### Package *robotFollower* 

* **centered_cmd** : makes two robots following a middle point between them contained in a trajectory
* **robotFollower.py** : makes a robot following a leader robot in Gazebo by using PID controllers  
* **leaderPath.py** : makes the leader robot following a pre-defided trajectory
* **utils** : regroup all the handmade functions needed in the package
* **README** : explain how to start the nodes of the package

# Useful commande

### Start the Gazebo project

If it is necessary, build the project and source the environment : 
```bash
cd ~/ROS2_Project
colcon build
source install/setup.bash
```

Then, enter the following command to start gazebo with the correct sdf file :

```bash
ros2 launch launch/MCAR.launch.py 
```
Now that Gazebo started using the right .sdf file, you just have to start the desired nodes.

### Start a node 

First go to the correct folder :  
*Here we go to the command folder inside the main folder because it is where we put all our ROS packages*  

```bash
cd ~/ROS2_Project/commande/
```

Rebuild if it is necessary : 

```bash
colcon build
```

Source the terminal : 

```bash
source install/setup.bash
```

Finally, enter and adapt the following command :
```bash
ros2 run packagename packagenode
``` 

Here find an example for starting the node for the leader car in the case of the shadowing part : 
```bash
cd ~/ROS2_Project/commande/
colcon build
source install/setup.bash
ros2 run robotFollower robotLeader
```

### Sources the environment for the project file
```bash
source ~/ROS2_Project/install/setup.bash
```

### Typo check of a sdf file
```bash
ign sdf -k diff_drive.sdf
```


# Git and Github

## Installing Git

To check if you have git, check the version with the following command :  
```bash
git --version
```
 If no message is written, then you need to install git.  
 For that, start by the following command :  
```bash
 sudo apt update  
```
 Then :  
```bash
 sudo apt install git  
```
 You can check if git is well installed by checking the version like we did in a first time.


## Get the project from Github / Clone the project

Here, we use the HTTPS link.  
Find here the link of the project :  
```bash
 https://github.com/romaindgc/ROS2_Project.git
```
 Next, thanks to your terminal, go where you want to put the folder.  
 For example, it can be at your *root*, use the next command to go there :  
 
```bash
 cd
```

Then, use the following command for cloning the repository in your machine :

```bash
git clone https://github.com/romaindgc/ROS2_Project.git
```

 ## Put your updates on GitHub 

 ### Token

 If you have already created a token, you can directly go to the next step.  
 Hence, go to this website for creating a new github token (classic) :  

 > https://github.com/settings/tokens

### Configur Git

If you’ve never configured Git on your machine, start by setting your name and email (which will be attached to your commits). Use these commands in the terminal :

```bash
git config --global user.name "Your Name"
```
```bash
git config --global user.email "youremail@example.com"
```

### Check for changes

Before pushing your changes, check which files have been modified using : 

```bash
git status
```

You can also use this command to see the progression in the *pushing process*.  

### Add changes to the staging area  

You need to add the modified or created files to the staging area before commiting :

* To add all changed files : 
```bash
 git add .
```
* To add specific files :
 ```bash
 git add *path/namefile*
```

### Commit the changes

Once files are added to the staging area, create a commit to save the changes with a descriptive message:  

```bash
git commit -m "Description of your changes"
```

### Push the changes to GitHub

Finally, push the changes to the remote repository on GitHub using :

```bash
git push origin master
```

*In our case, **master** refers to the main branch of the repository. If you are working in another branc, replace **master** with 
the appropriate branch name.*

## Import the lastest version / Pull the repository

### Navigate to the local repository

Fist, navigate to the local repository.  
If you put it at your *root*, it should be located there : 

```bash
cd ~/ROS2_Project
```

### Pull the changes

Use the following command to pull the latest changes from the remote repository to your local repository :

```bash
git pull origin master
```

*In our case, **master** is the default name for the remote repository.*

### Resolve conflits (if any)

If there are changes both in your local repository and on GitHub, there might be merge conflicts. Git will notify you if there are any, and you will need to manually resolve them before completing the pull.  

Once you’ve resolved any conflicts, stage the changes, commit them, and finish the pull.
