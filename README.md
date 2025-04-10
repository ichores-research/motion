# motion


## Docker

```sh
docker run --rm -it --network host -e ROS_MASTER_URI=$ROS_MASTER_URI -e ROS_IP=$ROS_IP motion
```

## Files

- `pick.py` opposed to its name it just extends the arm and closes the gripper
- `pick_collision.py` same example but with collision avoidance (octomap and table)
- `motion_service.py` - production ROS service for Tiago motions