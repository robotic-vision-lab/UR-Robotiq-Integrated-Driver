#!/usr/bin/env sh

# courtesy of https://github.com/NVlabs/Deep_Object_Pose/blob/master/docker/run_dope_docker.sh
# script folder; https://stackoverflow.com/a/4774063

SCRIPT_DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

if [ -z "${CONTAINER_NAME}" ]; then
    CONTAINER_NAME=rvl-driver-container
fi

if [ -z "${HOST_CATKIN_WS}" ]; then
    HOST_CATKIN_WS=$(cd "$(dirname "$0")/../catkin_ws/src" >/dev/null 2>&1 ; pwd -P )
fi

# get docker container ID if exists
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`

if [ -z "${CONTAINER_ID}" ]; then
    # container DNE, creating one with either given name or default name (rvl-container)
    echo "Creating new RVL Integrated Driver container in background named ${CONTAINER_NAME}"
    sleep 1

    # creating the docker container
    # see https://docs.docker.com/engine/reference/run/ for more details
    docker run -t -d \
    --name ${CONTAINER_NAME} \
    --privileged \
    --shm-size 16G \
    -p 50000-50003:50000-50003 \
    -p 54321:54321 \
    -p 10000:10000 \
    -e "DISPLAY=${DISPLAY}" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "${HOST_CATKIN_WS}:/root/catkin_ws/src:rw" \
    rvl-ur-robotiq-driver:latest \
    bash

    # add convenient aliases
    docker cp ${SCRIPT_DIR}/.bash_aliases ${CONTAINER_NAME}:/root/.bash_aliases
else
    echo "Found RVL Driver container: ${CONTAINER_NAME}"
    # Check if the container is already running and start if necessary.
    if [ -z `docker ps -qf "name=^/${CONTAINER_NAME}$"` ]; then
        echo "${CONTAINER_NAME} container not running. Starting container..."
        docker start ${CONTAINER_ID}
        docker exec -it ${CONTAINER_ID} bash
    else
        echo "Attaching to running ${CONTAINER_NAME} container..."
        docker exec -it ${CONTAINER_ID} bash
    fi
fi