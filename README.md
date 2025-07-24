# motion

This repository provides a motion service for the Tiago robot, exposing a repertoire of actions such as `pick` and `prepare_robot`. It supports both ROS Noetic and Melodic distributions.

## Getting Started

### Building the Docker Image

For **ROS Noetic**:

```sh
docker build --rm -t motion .
```

For **ROS Melodic**:

```sh
docker build --rm -t motion -f Dockerfile.melodic .
```

### Running the Service

To start the motion service container:

```sh
docker run --rm -it --network=host \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    motion
```

Ensure that the environment variables `ROS_MASTER_URI` and `ROS_IP` are set appropriately.

## Example Usage

An example client is provided in `example.py`, demonstrating how to call the motion service (e.g., to pick up a banana). To try it out:

1. Copy the required banana assets into the `data/` directory (grasps in `obj_000010.npy` and mesh in `obj_000010.ply`).

2. Build the example Docker image:

        ```sh
        docker build --rm -t motion-example -f Dockerfile.example .
        ```


3. Run the example as needed.

---

For more details, refer to the code and comments in `example.py`.