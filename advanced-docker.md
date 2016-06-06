# PX4 Docker Containers

Docker containers are available that contain the complete PX4 development toolchain including Gazebo and ROS simulation:

  * **px4io/px4-dev**: toolchain including simulation
  * **px4io/px4-dev-ros**: toolchain including simulation and ROS (incl. MAVROS)

Dockerfiles and README can be found here: https://github.com/PX4/containers/tree/master/docker/px4-dev

They are build automatically on Docker Hub: https://hub.docker.com/u/px4io/

## Prerequisites

Install Docker from here https://docs.docker.com/installation/, preferably use one of the Docker-maintained package repositories to get the latest version.

Containers are currently only supported on Linux. If you don't have Linux you can run the container inside a virtual machine, see further down for more information. Do not use `boot2docker` with the default Linux image because it contains no X-Server.

## Use the Docker container

The following will run the Docker container including support for X forwarding which makes the simulation GUI available from inside the container. It also maps the directory `<local_src>` from your computer to `<container_src>` inside the container. Please see the Docker docs for more information on volume and network port mapping.

With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). If you connect/disconnect a device you have to restart the container.

```sh
# enable access to xhost from the container
xhost +

docker run -it --privileged \
    -v <local_src>:<container_src>:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    --name=container_name px4io/px4-dev bash
```

If everything went well you should be in a new bash shell now. Verify if everything works by running SITL for example:

```sh
cd <container_src>
make posix_sitl_default gazebo
```

If you exit the container, your changes are left in this container. The above “docker run” command can only be used to create a new container. To get back into this container simply do:

```sh
# start the container
sudo docker start container_name
# open a new bash shell in this container
sudo docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again. 

## Virtual machine support

Any recent Linux distribution should work.

The following configuration is tested:

  * OS X with VMWare Fusion and Ubuntu 14.04 (Docker container with GUI support on Parallels make the X-Server crash).

**Memory**

Use at least 4GB memory for the virtual machine.

**Compilation problems**

If compilation fails with errors like this:

```
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

Try disabling parallel builds.

**Allow Docker Control from the VM Host**

Edit `/etc/defaults/docker` and add this line:

```
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

You can then control docker from your host OS:

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# run some docker command to see if it works, e.g. ps
docker ps
```

## Legacy

The ROS multiplatform containers are not maintained anymore: https://github.com/PX4/containers/tree/master/docker/ros-indigo
