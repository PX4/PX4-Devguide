# PX4 Middleware

The PX4 Middleware consists primarily of device drivers for embedded sensors and a publish-subscribe based middleware to connect these sensors to applications running the [flight controls](concept-flight-stack.md).

The use of the publish-subscribe scheme means that:

  * The system is reactive: It will update instantly when new data is available
  * It is running fully parallelized
  * A system component can consume data from anywhere in a thread-safe fashion
