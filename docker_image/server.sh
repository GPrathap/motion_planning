DOCKER_IMG="prathap/drone_node:v4"
DOCKER_CONATINER_NAME="ros_trajectory_tracker"
HOME_DIRECTORY="/home/$USER/ros_trajectory_tracker"
mkdir -p $HOME_DIRECTORY

docker_run() {
  # if ant_boy_home directory presents
  if [ -d "$HOME_DIRECTORY" ]; then
    # docker run
    CONTAINER_ID=$(docker run -it --detach \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e DISPLAY=$DISPLAY  \
        -e QT_X11_NO_MITSHM=1 \
        --net=host \
        --privileged \
        --name $DOCKER_CONATINER_NAME \
      $DOCKER_IMG)

    # X forwarding
    host=$(docker inspect --format='{{ .Config.Hostname }}' $DOCKER_CONATINER_NAME)
    xhost +local:$host

    # check if the docker is running
    if [ "$(docker inspect --format='{{.State.Running}}' $CONTAINER_ID)" = "true" ]; then
      echo "$DOCKER_IMG startup completed
      $DOCKER_IMG has been started, enter using: bash ant_boy_docker.sh enter"
    else
      echo "ERROR: Failed to start $DOCKER_IMG"
    fi
  else
    echo "ant_boy_home directory is not available
    create a directory 'ant_boy_home' in '/home/$USER'"
  fi
}

docker_enter() {
  # docker exec
  xhost +
  docker exec -ti  $DOCKER_CONATINER_NAME bash -li
}

docker_start() {
  # docker exec
  xhost +
  docker start $DOCKER_CONATINER_NAME
}


docker_stop() {
  docker stop -t 0 $DOCKER_CONATINER_NAME
}

if [ "$1" = "run" ]; then
  # if container is already Running
  if docker ps --format '{{.Names}}' | grep -q $DOCKER_CONATINER_NAME; then
    # if requested to restart
    if [ "$2" = "-f" ]; then
      # stop the container
      docker_stop
    else
      echo "ERROR: $DOCKER_IMG is already running.
      Use bash ros_node_docker.sh enter to enter
      or
      Use bash ros_node_docker.sh start -f to restart."
      # exit from here
      exit
    fi
  fi
  echo "Starting $DOCKER_IMG container"
  docker_run
elif [ "$1" = "start" ]
then
  docker_start
elif [ "$1" = "enter" ]
then
  docker_enter
elif [ "$1" = "stop" ]
then
  docker_stop
else
  echo "Options:
  --help     Show this message and exit.

Commands:
  run  Start ros_node docker environment.
  start  Start ros_node docker environment.
  enter  Enter ros_node docker environment.
  stop   Stop ros_node docker environment."
fi

