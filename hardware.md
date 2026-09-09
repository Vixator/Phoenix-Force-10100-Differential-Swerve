Drivetrain motors & gearing

4x goBILDA 5203-2402-0005 Yellow Jacket motors (5.2:1 planetary, 1150 RPM no-load, 145.1 encoder ticks/rev at gearbox output — not the raw 28 ticks/rev at the motor shaft)
Control Hub ports: left pod = ports 0 (left motor) & 1 (right motor); right pod = ports 2 & 3
Steering transmission (per motor): 1:1 bevel pair → 16T gear → 54T gear (differential stage) → both 54T gears rigidly tied to 50T bevel gears (top/bottom) → single 19T ring gear driving the 60mm wheel
Confirmed forward-drive polarity: both motors on a module powered red-to-red/black-to-black drives that module forward
Confirmed steering polarity: right motor alone (red-to-red) spins the pod counterclockwise viewed from above (this took a couple of rounds to nail down correctly — the module wiring is symmetric between left/right pods)

Geometry

Track width (module-to-module, center to center): 362.96mm, both pods mounted directly left/right of robot center
60mm wheels

Odometry — goBILDA Pinpoint

Hardware map name "pinpoint", I2C port 0 on Control Hub
Mounted flat, local X → robot-right, local Y → robot-forward
goBILDA 4-bar dead wheel pods for X/Y, plugged into respective Pinpoint ports
Heading polarity, odometry pod direction sign, and exact mounting offset were explicitly deferred — flagged as needing a guided commissioning/test OpMode later, not yet verified

Teleop control scheme (as specified, now implemented)

Left stick angle → field-relative travel direction; left stick magnitude → speed (no trigger-based translation)
Right stick X → rotation about the pod midpoint (right = clockwise viewed from above); releasing it holds current heading (active heading hold), doesn't snap back to field-forward
Field-centric zero reference captured at Start (after Pinpoint calibrates stationary during INIT); robot begins aligned forward at the bottom-right corner of the field

Constraints/status

No current limits — steering targets azimuth as fast as possible, wheel speed tracks commanded speed as closely as possible
All PIDF/PD gains are unvalidated placeholders, meant to be tuned live later
FTC Dashboard isn't installed yet — tuning fields are public/static in the meantime
Session ended with the teleop class cleaned up, one implementation only (no leftover legacy trigger-drive or robot-centric fallback path), APK build passing, and ~342k simulated regression assertions passing — but explicitly not validated against real hardware/tuning yet

Still open per the session's own "remaining for you" list: Pinpoint heading polarity, odometry pod direction sign, exact mounting offset, and all physical PIDF/deadband/slew tuning.