# Docker Instruction

## 1. Build the Docker image

```
docker build -t rvl-ur-robotiq:latest .
```

`sudo` might be required if you did not complete [Linux Post-Installation Instructions](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

## 2. Create a container

```
docker run \
--rm \
--tty \
--interactive \
--network host \
--privileged \
--name rvl-ur-robotiq-container \
--env DISPLAY=$DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume [path_to_repository]/colcon_ws:/root/colcon_ws \
rvl-ur-robotiq:latest \
bash
```

Replace `[path_to_repository]/colcon_ws` with full path to repository directory on your machine e.g. `/home/user/UR-Robotiq-Integrated-Driver/colcon_ws`. You can also use `$(pwd)` to get current directory e.g. `$(pwd)/colcon_ws` of the shell.

## 3. Attach additional terminal to container

```
docker exec -it rvl-ur-robotiq-container bash
```

## 4. Continue with [Quick Start](../README.md)
