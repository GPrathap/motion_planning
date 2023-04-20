# To give executable permission 
    
    chmod +x server.sh

# To see the available commands 
    
    ./server.sh --help

# To run the docker container  
    ./server.sh run 

# To start the docker container 
    ./server.sh start 

# To enter the docker container 
    ./server.sh enter

# ROS1 workspace configuration 
    
    cd /root/catkin_ws/src
    git clone git@github.com:GPrathap/motion_planning.git
    cd /root/catkin_ws/
    catkin build

# Run the ros node 
    
    roslaunch motion_planner  motion_planner.launch


