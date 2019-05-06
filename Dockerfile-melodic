FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y \
    build-essential python-catkin-tools

# Create ROS workspace
RUN  mkdir -p /ws/src

RUN /bin/bash -c "cd /ws && \
    source /opt/ros/melodic/setup.bash && \
    catkin init && \
    catkin config --install -j 1 -p 1 && \
    catkin build --limit-status-rate 0.1 --no-notify"

COPY . /ws/src/sick_ldmrs_laser
RUN git clone -b master https://github.com/SICKAG/libsick_ldmrs.git /ws/src/libsick_ldmrs

WORKDIR /ws

# Use rosdep to install all dependencies (including ROS itself)
RUN rosdep install --from-paths src -i -y --rosdistro melodic

RUN /bin/bash -c "source /ws/devel/setup.bash && \
    catkin build --limit-status-rate 0.1 --no-notify -DCMAKE_BUILD_TYPE=Release && \
    catkin build --limit-status-rate 0.1 --no-notify sick_ldmrs_description sick_ldmrs_driver sick_ldmrs_laser sick_ldmrs_msgs sick_ldmrs_tools --make-args tests"
