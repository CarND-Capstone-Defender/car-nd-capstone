## Introduction
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).


## Team Details
  * Team Name:
    * Defender
  * Team Lead : Peter Barry :Udacity: pete.barry@gmail.com :  slack @peter.barry
  * Team Members
    *  Antonia Reiter   : Udacity: antonia.reiter@gmx.de : Slack @antonia
    *  Ahmed Zikry   : Udaciy: ahmedzikry89@gmail.com : Slack @zika
    *  Waqas Malik   : Udacity: malik-waqas@hotmail.com : Slack @waqas

### Instructions
```bash
 git clone https://github.com/CarND-Capstone-Defender/car-nd-capstone.git
 ```

 To Build the docker instance ( note we updated some versions of of packages in the requirements.txt)

 call following sequence in the shell
 ```bash
./builddocker.sh
./rundocker.sh
 ```


## Usage on simulator
```bash

  ./runme_sim.sh
  ```

## Usage on Carla
```bash
  ./runme_site.sh
  ```

### Stadard  Installation wihhout startup scripts.
  [Install Docker](https://docs.docker.com/engine/installation/)

  Build the docker container
  ```bash
  sudo docker build . -t capstone
  ```

  Run the docker file
  ```bash
  docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
  ```

  ### Usage

  1. Clone the project repository
  ```bash
  git clone https://github.com/CarND-Capstone-Defender/car-nd-capstone.git
  ```

  2. Install python dependencies
  ```bash
  cd CarND-Capstone
  pip install -r requirements.txt
  ```
  3. Make and run styx
  ```bash
  cd ros
  catkin_make
  source devel/setup.sh
  roslaunch launch/styx.launch
  ```
  4. Run the simulator


### Simulator Installation

*  [Simulator ](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2)

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)
