# ROS2 Project

## Install Git

To check if you have git, check the version with the following command :  
```cpp
git --version
```
 If no message is written, then you need to install git.  
 For that, start by the following command :  
```cpp
 sudo apt update  
```
 Then :  
```cpp
 sudo apt install git  
```
 You can check if git is well installed by checking the version like we did in a first time.

## Get the project from Github / Clone the project

Here, we use the HTTPS link.  
Find here the link of the project :  
```cpp
 https://github.com/romaindgc/ROS2_Project.git
```
 Next, thanks to your terminal, go where you want to put the folder.  
 For example, it can be at your *root*, use the next command to go there :  
 
```cpp
 cd
```

Then, use the following command for cloning the repository in your machine :

```cpp
git clone https://github.com/romaindgc/ROS2_Project.git
```

## Run the project

Go inside the project folder *ros_gz_sim_demos* with the following command :  

```cpp
 cd ~\ROS2_Project
``` 

Rebuild the project to be sure :

```cpp
 colcon build
```

Next, enter this command : 

```cpp
 source install/setup.bash
```

 Finally, run the following command :

```cpp
 ros2 launch ros2_mcar diff_drive.launch.py
```

 ## Put your updates on GitHub 

 ### Token

 If you have already created a token, you can directly go to the next step.  
 Hence, go to this website for creating a new github token (classic) :  

 > https://github.com/settings/tokens

### Configur Git

If you’ve never configured Git on your machine, start by setting your name and email (which will be attached to your commits). Use these commands in the terminal :

```cpp
git config --global user.name "Your Name"
```
```cpp
git config --global user.email "youremail@example.com"
```

### Check for changes

Before pushing your changes, check which files have been modified using : 

```cpp
git status
```

You can also use this command to see the progression in the *pushing process*.  

### Add changes to the staging area  

You need to add the modified or created files to the staging area before commiting :

* To add all changed files : 
```cpp
 git add .
```
* To add specific files :
 ```cpp
 git add *path/namefile*
```

### Commit the changes

Once files are added to the staging area, create a commit to save the changes with a descriptive message:  

```cpp
git commit -m "Description of your changes"
```

### Push the changes to GitHub

Finally, push the changes to the remote repository on GitHub using :

```cpp
git push origin master
```

*In our case, **master** refers to the main branch of the repository. If you are working in another branc, replace **master** with 
the appropriate branch name.*

## Import the lastest version / Pull the repository

### Navigate to the local repository

Fist, navigate to the local repository.  
If you put it at your *root*, it should be located there : 

```cpp
cd ~/ROS2_Project
```

### Pull the changes

Use the following command to pull the latest changes from the remote repository to your local repository :

```cpp
git pull origin master
```

*In our case, **master** is the default name for the remote repository.*

### Resolve conflits (if any)

If there are changes both in your local repository and on GitHub, there might be merge conflicts. Git will notify you if there are any, and you will need to manually resolve them before completing the pull.  

Once you’ve resolved any conflicts, stage the changes, commit them, and finish the pull.
