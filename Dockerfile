# Udacity capstone project dockerfile
FROM ros:kinetic-robot
LABEL maintainer="olala7846@gmail.com"

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# setup rosdep
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
RUN apt-get install -y python-pip
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

# install required ros dependencies
RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros

# Setup kinetic Version in shell environment
RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_PACKAGE_PATH=/opt/ros/kinetic/share/:/capstone/ros/src/" >> /root/.bashrc

# upgrade the pip and pillow packages
# reason for pip was https://github.com/pdfminer/pdfminer.six/issues/27
# reason for pillow was https://discussions.udacity.com/t/capstone-project-message-error-image-publisher/401756/11
RUN pip install --upgrade pip
RUN pip install --upgrade pillow 


# necessary for trafficlight detection with mobilenet
RUN apt-get install -y protobuf-compiler
RUN pip install --upgrade matplotlib
RUN apt-get install -y python-tk

# ensure that the dependencies are always up to date
RUN rosdep update
CMD ./initCapstone.sh ; /bin/bash