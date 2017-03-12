# Land Detector Configuration

The land detector is a dynamic vehicle model representing key vehicle states such as landed and ground contact.

## Multicopter Land Detector Configuration

The complete set of parameters is available in the QGroundControl parameter editor under the `LNDMC` prefix. The key parameters that might differ per airframe are these:

* `LNDMC_MAN_DWNTHR` - the threshold \(in percent, default is 15%\) for how much manual throttle is allowed to be considering the state as landed. Systems with very high thrust-to-weight ratios like racers might need a lower setting here \(e.g. 8%\).
* `MPC_THR_HOVER` - the hover throttle of the system \(in percent, default is 50%\). It is important to set this correctly as it does not only make the altitude control more accurate, but also ensures correct land detection. A racer or a big camera drone without payload mounted might need a much lower setting \(e.g. 35%\).
* `LNDMC_POS_UPTHR` - throttle level to trigger takeoff \(in percent, default is 65%\). If the pilot raises above this threshold the system will attempt to take off. This value should be greater than the hover throttle.



## Fixed Wing Land Detector Configuration



