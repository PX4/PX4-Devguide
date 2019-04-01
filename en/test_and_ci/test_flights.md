# Tests Flights

Test flights are important for quality assurance. 
The Dronecode test team can help review (test flight) your pull requests and provide feedback and logs.

## How to Request Test Flights

* Add a complete and thorough description of your changes in the pull request
* Tag the test team in a comment using **@PX4/testflight** 
* Wait for feedback from the test team
* The test team will [add your PR/issue to their queue](https://github.com/PX4/Firmware/projects/18)

## Response Times

* Multi-Copter: up to 48 hours (typically within 24 hours)
* VTOL, Fixed Wing: up to 4 days (typically 2 days)

## Test Cards

The tests performed for each platform are linked below: 

* [MC_01 - Manual modes](../test_cards/mc_01_manual_modes.md)
* [MC_02 - Full Autonomous](../test_cards/mc_02_full_autonomous.md)
* [MC_03 - Auto Manual Mix](../test_cards/mc_03_auto_manual_mix.md)
* [MC_04 - Failsafe Testing](../test_cards/mc_04_failsafe_testing.md)
* [MC_05 - Indoor Flight (Manual Modes)](../test_cards/mc_05_indoor_flight_manual_modes.md)


## Test Vehicles/Autopilots

Multicopter

Frame | Flight Controller | UUID
--- | --- | ---
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhawk Mini | 002400283335510A33373538 (f450-v3)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhawk 1 | 000100000000363533353336510900500021 (f450-v3)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Cube (Pixhawk 2.1) | 00010000000033343537313751050040001c (F450 Pixhawk v2 cube)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixracer | 00010000000037373430333551170037002a (F450-Pixracer)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhawk 3 Pro | 000100000000303236353136510500180036 (Pixhawk pro)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhack V3 | 003200293036511638363834 (f450-v5-m)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhawk 4 | 000200000000383339333038510700320016 (F450-v5)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) | Pixhawk 4 Mini  | 0002000000003432333830385115003a0033 (F450-v5-m)
[DJI F450](http://px4.io/portfolio/dji-flamewheel-450/) UAVCAN | Pixhawk 4 | 000200000000323634353237511800200021 (F450-Pixhawk4)
NXP Vehicle (TBD) | RDDRONE-FMUK66 | TBD

Fixed Wing

Frame | Flight Controller | UUID
--- | --- | ---
Phantom Wing | Pixhawk 1 | 0001000000003035333330365104003c0020 (f450-v2)


VTOL

Frame | Flight Controller | UUID
--- | --- | ---
[Convergence VTOL](https://docs.px4.io/en/frames_vtol/vtol_tiltrotor_eflite_convergence_pixfalcon.html) | Pixhawk 4 Mini | 000200000000343233383038511500350039 (vtol-v5-m)
[Delta Quad Pro](https://px4.io/portfolio/deltaquad-vtol/) | Dropix | 0001000000003437393931375114004c0042 (delta-v2)

