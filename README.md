# openai_crazyflie

This project describes how to implement Reinforcement Learning in the swarm of the Crazyflie robots system. 
It is built upon the [openai_ros](https://bitbucket.org/theconstructcore/openai_ros/src/kinetic-devel/) and [CrazyS](https://github.com/gsilano/CrazyS) repositories. Note that current repo is only tested in Unbuntu 18.04 and Gazebo9.

## Step 0

- Download the openai_ros related files:

        cd ~/catkin_ws/src
        git clone https://bitbucket.org/theconstructcore/openai_examples_projects.git
        git clone https://bitbucket.org/theconstructcore/openai_ros.git
        cd openai_ros
        git checkout version2
        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

- Download the Crazyflie related files: Please follow the steps described in [CrazyS](https://github.com/gsilano/CrazyS). 

## Step 1 Creates environments for Crazyflie robots.

- Copy the file `cf_env.py` to `openai_ros/openai_ros/src/openai_ros/robot_envs`

- Add the folder `Crazyflie` to `openai_ros/openai_ros/src/openai_ros/task_envs`

- In `openai_ros/openai_ros/src/openai_ros/task_envs`, add the following code at line 207.

    elif task_env == 'Crazyflie-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.Crazyflie.cf_goto:CrazyflieGotoEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.Crazyflie import cf_goto

## Step 2 Create the learning algorithm and main file

Use the `catkin_create_pkg` create the folder that contains files as in folder `crazyflie_openai`.

## Step 3 Edit the initial position of robots

To change the number of robots and set the initial position, go to file `CrazyS/rotors_gazebo/resource/spline_trajectory.yaml` to change.

## Step 4 First test

To run the code, firstly run `roslaunch rotors_gazebo crazyflie2_swarm_hovering_example.launch`.
Then run `roslaunch crazyflie_openai cf_start_training.launch`.
