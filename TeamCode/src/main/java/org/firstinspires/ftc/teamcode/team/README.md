# Team starter code

Students tune the robot and write the autonomous. `pedro/Constants.java` holds your tuned numbers.

| File | Who | What it is |
|---|---|---|
| `pedro/Constants.java` | students, after each tuning run | Hardware names and tuned numbers. `TUNED` must be `true` before anything runs. |
| `team/LimelightHelper.java` | provided | Wraps the Limelight. Students call `relocalize(follower)`. |
| `team/Poses.java` | students | Named field spots. Two examples to start. |
| `team/TemplateAuto.java` | students | State machine with one worked state. Students write the rest. |

## Coordinate frame

Origin at the field corner, inches, heading in degrees (in `Poses`). This is the Pedro
Visualizer's frame, so students copy its numbers directly. The Limelight reports from the field
center in meters, so `LimelightHelper` converts (meters to inches, then +72 in on x and y).

## First-time setup (students)

1. Name the devices in the Robot Configuration: `pinpoint`, `limelight`, and the four drive motors.
2. Run the Tuning OpMode. Paste each tuner's output over the matching block in `Constants.java`.
   Set `TUNED = true`.
3. Limelight web UI: load the AprilTag pipeline for the current game and set the camera position.
4. Check the frame once: place the robot at a known pose, call `relocalize` and compare the
   telemetry pose with where the robot really is. If x and y look swapped or mirrored, the heading
   frame needs an offset in `LimelightHelper`.

## Before each competition

Tag the known-good version: `git tag comp-<event>`. Students experiment on branches.
