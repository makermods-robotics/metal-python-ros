#!/bin/bash

DEV_CONTAINER="metal_ros2"

function restart_stopped_container {
    if docker ps -f status=exited -f name="${DEV_CONTAINER}" | grep "${DEV_CONTAINER}"; then
        docker start "${DEV_CONTAINER}"
    fi
}

xhost +local:root 1>/dev/null 2>&1

restart_stopped_container

docker exec -u 0 -it ${DEV_CONTAINER} /bin/bash

xhost -local:root 1>/dev/null 2>&1