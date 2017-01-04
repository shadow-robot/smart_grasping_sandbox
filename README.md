# smart_grasping_sandbox

This is a public simulation sandbox for [Shadow's Smart Grasping System](https://www.shadowrobot.com/shadow-smart-grasping-system/). We're aiming to provide you with a simplified simulation environment to play with different challenges in an autonomous pick and place problem.

This stack contains: 
* **fh_description**: the urdf description of the robot
* **smart_grasp_moveit_config**: a MoveIt! config for the motion planning
* **smart_grasping_sandbox**: the main point of entrance, includes the launch file and a [smart_grasping_sandbox.py](smart_grasping_sandbox/scripts/smart_grasping_sandbox.py) script that goes through a pick routine to help you get started.

## Building the docker container
First clone the repository. Then go to the root of the cloned repository (where the README, Dockerfile, etc... are) and run:

```
docker build -t smart-grasping-sandbox .
```

You can now start the container:

```
docker run -it --name sgs --entrypoint /bin/bash -p 8080:8080 -p 7681:7681 -p 8181:8181 shadowrobot/smart-grasping-sandbox
```

Once in the container, you can fire up the simulation with:

```
/entrypoint.sh
```

Then connect your local browser to [localhost:8080](http://localhost:8080) to see the simulation and [localhost:8181](http://localhost:8181) for a web ide in the docker workspace.

You can fire up the grasping script with:

```
python /workspace/src/smart_grasping_sandbox/smart_grasping_sandbox/scripts/smart_grasping_sandbox.py
```
