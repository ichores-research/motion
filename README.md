# motion

This repository provides a motion service for the Tiago robot, exposing a repertoire of actions such as `pick` and `prepare_robot`. It supports both ROS Noetic and Melodic distributions.

This service requires table_surface_extractor service to be running -> https://github.com/ichores-research/table_plane_extractor.

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

1. Build the example Docker image:

        ```sh
        docker build --rm -t motion-example -f Dockerfile.example .
        ```

2. Copy the required banana assets into the `data/` directory.

3. Run the example as needed.

---

For more details, refer to the code and comments in `example.py`.