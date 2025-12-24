# Franka Motion Planning 
Authors: Conor Hayes, Kyuwon Weon, Amber Handal, Tianhao Zhang

This ROS 2 package demonstrates autonomous pick-and-place motion planning for a Franka Emika Panda robotic arm using MoveIt 2. In the RViz simulation, the robot detects and avoids a fixed obstacle on a table, approaches a rectangular object, grasps it with its gripper, and places it on the opposite side without collisions.

## Quickstart
1. Use `ros2 launch motion_planner pickplace.launch.xml` to start RViz with the Franka arm and planning scene setup.
2. Use `ros2 service call /ready std_srvs/srv/Trigger {}` to return to a ready state and `ros2 service call /pick std_srvs/srv/Trigger {}` to commence the simulation (avoid the obstacle, pick up the block, and release it on the opposite side of the obstacle).
3. Here is a video of the simulation in Rviz:

[RViz Screencast](https://github.com/user-attachments/assets/f4ff69f7-5683-4d89-9458-c81dae230dd8)

   
5. Here is a video of the Franka arm executing this demo from `pickplace.launch.xml`:

[Live Demo](https://github.com/user-attachments/assets/159ed679-2d91-4476-b1d2-eb6ef0bf2057)

## Developer Instructions
This repo contains a pre-commit hook that performs lint checks before you're allowed to commit code, and also auto-formats some of those errors for you (i.e. replacing double quotes with single quotes). This is so we don't have to go back and fight with with ament_lint for a million years like we did in the previous project.

It uses the python `pre-commit` framework + the `ruff` formatter+linter to do this; see docs for [pre-commit](https://pre-commit.com/) and [ruff](https://docs.astral.sh/ruff/).

In order to set it up, do the following:

```bash
# install the pre-commit program
sudo apt install pre-commit

# install the pre-commit hooks to the repo (as configured in .pre-commit-config.yaml)
pre-commit install
```

All done! Now every time you commit, it will run lint checks + do some autoformatting to make sure that we 
stick to the ROS2 style guidelines (mostly. it doesn't do everything for us/check everything).
