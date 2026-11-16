# Team starter code

Students tune the robot and write the autonomous. `pedro/Constants.java` holds your tuned numbers.

| File | Who | What it is |
|---|---|---|
| `pedro/Constants.java` | students, after each tuning run | Hardware names and tuned numbers. `TUNED` must be `true` before anything runs. |
| `team/HuskyLensHelper.java` | provided, `relocalize` is a TODO | Wraps the HuskyLens camera: `blocks()`, `seesTarget()`, `blockWithId(id)`. |
| `team/HuskyLensTest.java` | provided | TeleOp that prints everything the HuskyLens sees. |
| `team/Poses.java` | students | Named field spots. Two examples to start. |
| `team/TemplateAuto.java` | students | State machine with one worked state. Students write the rest. |

## Coordinate frame

Origin at the field corner, inches, heading in degrees (in `Poses`). This is the Pedro
Visualizer's frame, so students copy its numbers directly. The HuskyLens does not report a field
position, only where things are in its 320 x 240 picture. Turning that into a field pose is the
`relocalize` TODO in `HuskyLensHelper`.

## First-time setup (students)

1. Name the devices in the Robot Configuration: `pinpoint`, `huskylens` (I2C port, device type HuskyLens), and the four drive motors.
2. Run the Tuning OpMode. Paste each tuner's output over the matching block in `Constants.java`.
   Set `TUNED = true`.
3. HuskyLens: on the camera, set General Settings > Protocol Type to I2C. Pick the algorithm in
   `TuningConfig.huskyLensAlgorithm` (default AprilTags). Run `HuskyLens Test` to check it sees things.
4. TODO: once `relocalize` is written, place the robot at a known pose, call it and compare the
   telemetry pose with where the robot really is.

## Before each competition

Tag the known-good version: `git tag comp-<event>`. Students experiment on branches.
