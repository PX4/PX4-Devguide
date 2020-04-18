# Multi-Vehicle Simulation with FlightGear

This topic explains how to simulate multiple vehicles using FlightGear in SITL.
All vehicle instances have parameters defined by their startup scripts.

> **Tip** This is the most environmentally realistic way to simulate multiple vehicles running PX4.
  It is suitable for testing multi-vehicle support in *QGroundControl* (or the [MAVSDK](https://mavsdk.mavlink.io/), etc.).
  [Multi-Vehicle Simulation with Gazebo](../simulation/multi-vehicle-simulation.md) should be used for swarm simulations with many vehicles or for testing features like computer vision that are only supported by Gazebo.


## How to Start Multiple Instances

To start multiple instances (on separate ports and IDs):

 1. Checkout the [PX4 branch which supports multiple vehicles](https://github.com/ThunderFly-aerospace/PX4Firmware/tree/flightgear-multi)
   ```
   git clone https://github.com/ThunderFly-aerospace/PX4Firmware.git
   cd PX4Firmware
   git checkout flightgear-multi  
   ```
 2. Build the PX4 Firmware by standard toolchain with FlightGear installed

 3. Start the first instance using the [predefined scripts](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge/tree/master/scripts):

   ```
   cd ./Tools/flightgear_bridge/scripts
   ./vehicle1.sh
  ```
  
 4. Start subsequent instances by another script:

   ```
   ./vehicle2.sh
   ```

Each instance should have its own startup script. Each instance can be completely different vehicle type. For prepared scripts you should get the following view.

![Multi-vehicle simulation using PX4 SITL and FlightGear](../../assets/simulation/flightgear/flightgear-multi-vehicle-sitl.jpg)

Ground stations such as *QGroundControl* connect to all instances using the normal UDP port 14550 (all traffic goes to the same port).

The number of simultaneously runnig instances is limited mainly by computer resources. The FlightGear is single thread application, but aerodynamics solvers consume a lot of memory. Therefore splitting to multiple computers and using a [multiplayer server](http://wiki.flightgear.org/index.php?title=Howto:Multiplayer) is probably the required step to run many vehicle instances.

## Additional Resources

* See [Simulation](../simulation/README.md) for more information about the port configuration.
