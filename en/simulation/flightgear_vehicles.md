# FlightGear Vehicles

This topic lists/displays the vehicles supported by the PX4 [FlightGear](../simulation/flightgear.md) simulation and the `make` commands required to run them (the commands are run from terminal in the **Firmware** directory).

Supported vehicle types include: Plane, Autogyro, Rover.

> **Tip** For the full list of build targets run `make px4_sitl list_vmd_make_targets` (filter out those that start with `flightgear_`).

<span></span>
> **Note** The [FlightGear](../simulation/flightgear.md) page shows how to install FlightGear, how to enable global features and enable scenarios. This page is a summary for vehicle specific features.


## Standard Plane {#standard_plane}

The FlightGear has models for many planes.  The most suitable one for UAV development is currently the [Rascal RC plane](https://github.com/ThunderFly-aerospace/FlightGear-Rascal), which also exists in multiple variants.

![Rascal plane in FlightGear](../../assets/simulation/flightgear/vehicles/rascal110.jpg)


The variants differ by some features and mainly by using the different [FDM](http://wiki.flightgear.org/Flight_Dynamics_Model). All variants have a common feature selection table which could be activated by pressing "=" key on the computer keyboard.

There is a pop-up table that could be used for advanced features activation.

![Rascal plane FlightGear advanced options](../../assets/simulation/flightgear/vehicles/rascal_options.jpg)

The most relevant option is:

  * Smoke - generates a smoke trail to enhance the visibility of aircraft in the air. (Smoke and particles option needs to be activated in FG View -> rendering options -> Particles checkbox)
  * Trajectory markers - displays orthogonal markers along the trajectory of flight.

Trajectory markers show the absolute flight path in the world coordinates, and the smoke trail shows the relative path in the air mass.

### Rascal 110 YASim

The primary variant of Rascal model selected by

```sh
make px4_sitl_nolockstep flightgear_rascal
```

has a combustion piston-engine model. This results in a non-zero idle power causing a rotation of propeller on idle engine RPM.

### Rascal 110 Electric YASim

There also exists a variant with an electric engine.

```sh
make px4_sitl_nolockstep flightgear_rascal-electric
```

However, this variant needs the latest FlightGear code (sources at least from 26 April 2020). Otherwise, the FlightGear crashes because of an unexpected definition of electric engine.

### Rascal 110 JSBsim

JSBsim rascal variant currently does not have a direct make option but could be manually selected in the rascal.json configuration file, which is a part of [PX4-FlightGear-Bridge](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge)

In [this file](https://github.com/ThunderFly-aerospace/PX4-FlightGear-Bridge/blob/master/rascal.json#L2) the `Rascal110-YASim` should be changed to `Rascal110-JSBSim`.

## Autogyro {#autogyro}

The only UAV autogyro model supported by FlightGear is [TF-G1 Autogyro](https://github.com/ThunderFly-aerospace/TF-G1)

```sh
make px4_sitl_nolockstep flightgear_tf-g1
```

![TF-G1 in FlightGear](../../assets/simulation/flightgear/vehicles/tf-g1.jpg)



## Ackerman vehicle (UGV/Rover) {#ugv}

### TF-R1 Ground support Rover

This rover is equipped with a towing hitch. Therefore it could be used for aerotowing of other vehicles.  

```sh
make px4_sitl_nolockstep flightgear_tf-r1
```

![TF-R1 rover in FlightGear](../../assets/simulation/flightgear/vehicles/tf-r1_towing.jpg)


## Quadrotor {#quadrotor}

There is only an [incomplete multirotor model](https://github.com/ThunderFly-aerospace/FlightGear-TF-Mx1) which is numerically unstable and needs an additional work to become usable.

# Adding a New Vehicle

A new vehicle model needs to be included as a git submodule into [PX4-FlightGear-Bridge/models/](https://github.com/PX4/PX4-FlightGear-Bridge/tree/master/models) directory.  This directory contains an control channel definition [JSON file](https://github.com/PX4/PX4-FlightGear-Bridge/blob/master/models/rascal.json). 

```
{
    "FgModel":"Rascal110-YASim",
    "Url":"https://github.com/ThunderFly-aerospace/FlightGear-Rascal/archive/master.zip",
    "Controls": [
                ["5","/controls/flight/aileron","-1"],
                ["7","/controls/flight/elevator","-1"],
                ["2","/controls/flight/rudder","1"],
                ["4","/controls/engines/engine/throttle","1"]
                ]
}
```

The file content meaning is as follows: 

  * "FgModel" - a precise case sensitive name of the FlightGear model corresponding to "XXXX-set.xml" in the model directory (where XXXX is the model name).
  * "Url" is optional and it is not currently used. It is intended for future use to auto-download the models from web
  * "Controls" - the most important part of the process of adding a vehicle. This section contains the mapping between the PX4 mixer file and [FlightGear property tree](http://wiki.flightgear.org/Property_tree).
    * The first number in a list selects a PX4 mixer output.
    * Path string is a FlightGear variable location in the property tree.
    * The last number in a list is a multiplier, allowing inversion or scaling of mixer input.

After preparing all these files a new vehicle needs to be included in the PX4 make system. 

The configuration is in [/platforms/posix/cmake/sitl_target.cmake](https://github.com/PX4/Firmware/blob/c5341da8137f460c84f47f0e38293667ea69a6cb/platforms/posix/cmake/sitl_target.cmake#L164-L171) where a new vehicle json name should be added to the list. 




