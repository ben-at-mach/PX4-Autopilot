DroneCAN DSDL definitions that are not part of the DroneCAN/PX4 DSDL submodule
==============================================================================

`iridium/apnt` carries the output of the RockBLOCK APNT (Iridium PNT) receiver from a CAN node
(`iridium_apnt` driver + `uavcannode`) to the flight controller. It uses the vendor-specific
message range 20000..20999, block 20800..20809. Check the DSDL submodule for clashes before
adding more.

Each message mirrors the `iridium_apnt_*` uORB topic of the same name. Position and velocity
are also sent as the standard `uavcan.equipment.gnss.Fix2`, which carries the solution itself;
the messages here carry what Fix2 cannot (figure of merit, position mode, ENU covariance,
timing, per-burst data).

| ID    | Name                        | uORB topic              |
| ----- | --------------------------- | ----------------------- |
| 20800 | iridium.apnt.Position       | iridium_apnt_position   |
| 20801 | iridium.apnt.Velocity       | iridium_apnt_velocity   |
| 20802 | iridium.apnt.Pps            | iridium_apnt_pps        |
| 20803 | iridium.apnt.System         | iridium_apnt_system     |
| 20804 | iridium.apnt.Stats          | iridium_apnt_stats      |
| 20805 | iridium.apnt.Svi            | iridium_apnt_svi        |
| 20806 | iridium.apnt.Skyview        | iridium_apnt_skyview    |
| 20807 | iridium.apnt.Sentence       | iridium_apnt_sentence   |
| 20808 | iridium.apnt.Command        | iridium_apnt_command    |
| 20809 | iridium.apnt.Response       | iridium_apnt_response   |

Command and Response are broadcasts, not a DroneCAN service, because the receiver answers
asynchronously and libuavcan service servers must answer from inside the request callback.
