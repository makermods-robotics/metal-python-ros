#!/bin/bash

CAN_DEVICE="/dev/makermods_metal_can1"
CAN_INTERFACE="can1"

start_can() {
    echo "Starting slcand..."
    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    if [ $? -ne 0 ]; then
        echo "slcand failed to start"
        return 1
    fi
    echo "Configuring $CAN_INTERFACE..."

    sudo ifconfig $CAN_INTERFACE up
    sudo ip link set $CAN_INTERFACE txqueuelen 1000

    if [ $? -ne 0 ]; then
        return 1
    fi
    echo "$CAN_INTERFACE started successfully"
    return 0
}

check_can() {

    if ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then

        if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
            return 0
        else
            return 1
        fi
    else
        return 2
    fi
}

while true; do

    if check_can; then

        echo "CAN interface $CAN_INTERFACE is healthy"
    else

        echo "$CAN_INTERFACE is down, restarting..."

        sudo ip link set $CAN_INTERFACE down
        sleep 1

        if ! start_can; then
            echo "Failed to restart CAN. Check whether the USB-CAN adapter is connected."
        fi

    fi

    sleep 1

done