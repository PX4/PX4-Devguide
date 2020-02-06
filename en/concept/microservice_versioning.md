# Microservice Versioning - PX4 Firmware

Microservice versioning was implemented in PX4 under the assumption that PX4 will not initiate the microservice version handshake: That is, it will never send the `MAV_CMD_REQUEST_SERVICE_VERSION`. It will only respond. When PX4 receives a `MAV_CMD_REQUEST_SERVICE_VERSION`, it will perform 2 actions:

 - Respond with a `MAVLINK_SERVICE_VERSION` message
 - Use the command to determine which version of the service to use, and store that number for later

Whenever a microservice needs to determine which version of a service is being used, it can call `_mavlink->get_service_version_stream()->get_service_status(<service_id>)`, where `_mavlink` is the instance of `Mavlink`. This can have three possible results:

 - The handshake was already initiated by the other system, and there is already a selected service version. This is the expected result based on the assumption that PX4 will not initiate the handshake.
 - The handshake was already initiated by the other system, but there was no compatible version of the microservice available. This is unexpected. If the other system tried the microservice handshake and it resulted in no compatible versions, then it should not proceed to try and use that microservice anyway.
 - The handshake was never initiated for this microservice. This might happen if the other system is using old software that is not aware of microservice versioning. In this case, the microservice could act as it did before microservice versioning, or treat this as an error.

## Microservice version configuration

The set of currently-supported microservices and versions are stored as compile-time constants in `modules/mavlink/mavlink_service_versions.(h|cpp)`. These constants are used in the service version handshake to populate the minimum and maximum supported versions, and to finish the handshake and choose a version to use. When the firmware is updated to support a new microservice version, `mavlink_service_versions.cpp` should be updated as well, with the new version number.

## Implementation in Mission microservice

To demonstrate this functionality, the Mission microservice was chosen, because it already acts as though it has two separate versions:

 - Float mode, using `MISSION_ITEM`
 - Int mode, using `MISSION_ITEM_INT`

Currently, both of these types can be used. When PX4 receives a message in either of these types, it will switch to responding with the same type.

To demonstrate microservice versioning, the mission microservice was broken into 2 versions:

 1. Float mode
 2. Int mode

If the microservice version handshake has been performed, and a version was agreed upon, then the mission microservice will always use the selected type, even if the other system sends the 'wrong' type. This is not necessarily how it should behave in the final implementation, but it serves to demonstrate microservice versioning.

If the microservice version handshake was not performed, or if a version was not agreed upon, then the mission microservice will use the old behavior: Always respond with the same type that was sent/requested.