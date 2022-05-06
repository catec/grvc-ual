FROM ros:melodic-ros-base-bionic

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /catkin_ws/src

COPY . /catkin_ws/src/grvc_ual

RUN cd /catkin_ws/src/grvc_ual; \
    chmod +x configure.py; \
    ./configure.py

RUN chmod +x /catkin_ws/src/grvc_ual/ros_entrypoint.sh

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin_make; source devel/setup.bash'

CMD ["./catkin_ws/src/grvc_ual/ros_entrypoint.sh"]
