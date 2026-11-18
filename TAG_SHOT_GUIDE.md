# Tag shot guide: drive to a spot by the AprilTag, then shoot

When you're done, driver 1 holds **A** and the robot:

1. Finds the AprilTag with the Limelight.
2. Drives to a spot you chose, measured from that tag.
3. Spins the flywheel up to the right speed for that distance.
4. Feeds the balls once it is in place and the flywheel is at speed.

Do the steps in order. Every step ends with a **Checkpoint**. Don't move on until the
checkpoint passes.

## Before you start

- Finish `LIMELIGHT_GUIDE.md` through step 8. You need **Full 3D** on and the **camera position**
  entered. Without those, every number in this guide is wrong.
- Know how to run `Shooter Tuning` and read its telemetry.
- Bring a tape measure and a field tile with the tag on it (or the real field).

---

## Step 1: Make the OpMode

Make a new file `team/TagShot.java`. It **extends `BaseAuto`**, so `initRobot()` already sets up
the drive motors, the flywheel, the intake, the windmill and the Limelight for you.

```java
package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "Tag Shot", group = "Test")
public class TagShot extends BaseAuto {

    static final int TAG_ID = 0;   // step 2 tells you the real number

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        while (opModeIsActive()) {
            // steps 2 to 6 go here
            telemetry.update();
        }
        endAuto();
    }
}
```

**Checkpoint:** it builds, and "Tag Shot" shows up on the Driver Station under Test.

---

## Step 2: Read where the robot is, measured from the tag

The Limelight can report the robot's position in three ways. This guide uses **robot pose in
target space**: where the robot is, measured from the **tag**. The tag is the center (0, 0, 0).
That is the number we want, because the spot we're driving to is defined by the tag.

Add this method to the class. It returns the robot's position measured from our tag, or `null`
when the tag isn't in view.

```java
    /** Where the robot is, measured from tag TAG_ID. Null if that tag isn't in view. */
    private Pose3D robotFromTag() {
        LLResult result = limelight.getLatestResult();
        // getStaleness() is how old the picture is in milliseconds. Old pictures lie.
        if (result == null || !result.isValid() || result.getStaleness() > 100) {
            return null;
        }
        // The camera may see several tags. Only use ours.
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == TAG_ID) {
                return tag.getRobotPoseTargetSpace();
            }
        }
        return null;
    }
```

Inside the loop, print every number it gives you:

```java
            Pose3D robot = robotFromTag();
            if (robot == null) {
                telemetry.addData("Tag", "not seen");
            } else {
                Position p = robot.getPosition().toUnit(DistanceUnit.INCH);
                telemetry.addData("x (in)", "%.1f", p.x);
                telemetry.addData("y (in)", "%.1f", p.y);
                telemetry.addData("z (in)", "%.1f", p.z);
                telemetry.addData("yaw (deg)", "%.1f", robot.getOrientation().getYaw(AngleUnit.DEGREES));
            }
```

To find the tag's ID, open the Limelight web page (`http://limelight.local:5801`) with the robot
pointed at the tag. The ID is shown on the tag in the camera view. Put it in `TAG_ID`.

**Checkpoint:** point the robot at the tag. The numbers show up. Cover the camera and you see
"not seen".

---

## Step 3: Work out what the numbers mean

Don't trust anyone's word on which way is positive. Not this guide, not the docs. Measure it.
(Limelight's docs say x is to the tag's right, y is down, and z points straight out of the tag.
Check it anyway.)

Start with the robot facing the tag, about 4 feet away. Make **one** move at a time, then fill in
the table:

| I moved the robot...      | x did... | y did... | z did... | yaw did... |
|---------------------------|----------|----------|----------|------------|
| 1 foot farther from tag   |          |          |          |            |
| 1 foot to the robot's left|          |          |          |            |
| turned a little left      |          |          |          |            |

From the table, decide:

- Which number is **distance** from the tag? (It should change by about 12 when you move 1 foot back.)
- Which number is **sideways**?
- What does **yaw** read when the robot faces the tag straight on? Is it near 0, or near 180?

Now check the distance with the tape measure. Measure from the tag to the **center of the robot**,
because that's what the Limelight reports once the camera position is set.

**Checkpoint:** the distance number matches the tape measure within about an inch at 2, 4 and 6
feet. If it is off by the **same** amount every time, the camera position in the Limelight is
wrong (LIMELIGHT_GUIDE step 6). If it is off by **more the farther you go**, check the units you
entered there.

---

## Step 4: Pick your spot

Drive the robot by hand to the best shooting spot (use `Shooter Tuning` to make sure shots go in
from there). Run Tag Shot and copy the numbers into constants at the top of the class:

```java
    // ---- The spot (measured in step 4) ----
    static final double TARGET_DISTANCE_IN = 0;  // your distance number at the spot
    static final double TARGET_SIDEWAYS_IN = 0;  // your sideways number at the spot
    static final double TARGET_YAW_DEG = 0;      // yaw at the spot
```

This is the same trick CornerShot uses: put the robot where you want it and copy what the
sensor reads. No math needed.

**Checkpoint:** push the robot away and back to the spot. The numbers come back to within about
an inch and 2 degrees.

---

## Step 5: Drive to the spot

The idea is called **proportional control**:

1. **Error** = where I am − where I want to be.
2. **Power** = error × a small number (the **gain**).
3. Far away means big error, so it drives fast. Close means small error, so it drives slow. At
   the spot the error is zero, so it stops.

Add the gains and a drive method. The wheel formula is the one already checked on our robot (see
`BaseAuto.setWheels`).

```java
    // ---- How hard to drive ----
    static final double DRIVE_GAIN = 0.03;  // power per inch of error (10 in off = 0.3 power)
    static final double TURN_GAIN = 0.02;   // power per degree of error
    static final double MAX_POWER = 0.4;    // never faster than this
    static final double DONE_IN = 1.0;      // close enough, inches
    static final double DONE_DEG = 2.0;     // close enough, degrees

    /** forward > 0 drives forward, strafe > 0 strafes LEFT on our robot. */
    private void drive(double forward, double strafe, double turn) {
        frontLeftDrive.setPower(forward + strafe - turn);
        backLeftDrive.setPower(forward - strafe - turn);
        frontRightDrive.setPower(forward - strafe + turn);
        backRightDrive.setPower(forward + strafe + turn);
    }
```

Replace the loop from step 2 with this. Use **your** distance and sideways numbers from step 3
(this example uses z and x):

```java
            Pose3D robot = robotFromTag();
            if (robot == null || !gamepad1.a) {
                drive(0, 0, 0);   // lost the tag or let go of A: STOP. Never drive blind.
                telemetry.addData("Tag", robot == null ? "not seen" : "seen, hold A to go");
                telemetry.update();
                continue;
            }

            Position p = robot.getPosition().toUnit(DistanceUnit.INCH);
            double distance = p.z;   // <- your answer from step 3
            double sideways = p.x;   // <- your answer from step 3
            double yaw = robot.getOrientation().getYaw(AngleUnit.DEGREES);

            double distErr = distance - TARGET_DISTANCE_IN;
            double sideErr = sideways - TARGET_SIDEWAYS_IN;
            double yawErr = yaw - TARGET_YAW_DEG;
            // 179 and -179 degrees are only 2 degrees apart. Without these two lines
            // the robot would think it is 358 degrees off and spin the long way round.
            while (yawErr > 180) yawErr -= 360;
            while (yawErr < -180) yawErr += 360;

            boolean atSpot = Math.abs(distErr) < DONE_IN
                    && Math.abs(sideErr) < DONE_IN
                    && Math.abs(yawErr) < DONE_DEG;

            if (atSpot) {
                drive(0, 0, 0);
            } else {
                drive(Range.clip(distErr * DRIVE_GAIN, -MAX_POWER, MAX_POWER),
                      Range.clip(sideErr * DRIVE_GAIN, -MAX_POWER, MAX_POWER),
                      Range.clip(yawErr * TURN_GAIN, -MAX_POWER, MAX_POWER));
            }

            telemetry.addData("Errors", "%.1f in, %.1f in, %.1f deg", distErr, sideErr, yawErr);
            telemetry.addData("At spot", atSpot);
```

**Test one direction at a time.** Set the other two gains to 0 while you test.

1. **Turn only.** Start a little turned. Hold A. Does it turn toward the spot, or away?
2. **Forward only.** Start too far away. Does it come closer, or back off?
3. **Sideways only.** Start off to one side.

If one of them goes the **wrong way**, put a minus sign in front of that error in the `drive(...)`
call (for example `-distErr * DRIVE_GAIN`). Let go of A the moment it goes the wrong way.

Then tune:

- Too slow, or stops short? Raise the gain a little.
- Overshoots and wobbles back and forth? Lower the gain.

Start the robot mostly facing the tag (within about 30 degrees). The forward and sideways
numbers come from the tag's point of view, so if the robot is turned a lot, "forward" for the
robot isn't "toward the tag" anymore. Turning fixes itself as the robot gets close to the target
yaw.

**Checkpoint:** from 3 different starting spots, holding A brings the robot to the spot and
"At spot" turns true. Push the robot away while holding A and it comes back.

---

## Step 6: Spin up and shoot

`flywheel.spinTo(ticksPerSecond)` sets the speed. `flywheel.isReady()` is true once both motors
have held that speed for a moment. Call `spinTo` **every loop**.

Inside the `gamepad1.a` part of the loop, after the drive code:

```java
            flywheel.spinTo(SHOT_SPEED_TPS);   // spins up while it drives, so no time is wasted
            boolean shoot = atSpot && flywheel.isReady();
            intake.setPower(shoot ? TuningConfig.intakePower : 0);
            windmill.setPower(shoot ? TuningConfig.windmillPower : 0);
            telemetry.addData("Flywheel", "%.0f of %.0f tps", flywheel.getVelocity1(), flywheel.getTarget());
```

In the "lost the tag or let go of A" part, also stop everything:

```java
                flywheel.stop();
                intake.setPower(0);
                windmill.setPower(0);
```

For `SHOT_SPEED_TPS`, use the speed that worked at your spot in step 4. `Shooter Tuning` shows it as
**Target (tps)**. Add it as a constant.

**Checkpoint:** hold A. The robot drives to the spot, the flywheel spins up on the way, and the
balls feed only after it stops **and** the flywheel is ready. The shots go in.

You can stop here if the robot always drives to the **same** spot: same distance every time, so
it needs the same speed every time. Step 7 is for shooting from **different** distances.

---

## Step 7: Choose the speed from the distance

### The idea: speed is a ratio of distance

The simplest guess is:

```
speed = k × distance
```

Twice as far, twice as fast. You only need one number, `k`. It's a good guess to **test**, but
there are two reasons it probably won't hold exactly:

1. **What happens at distance 0?** The ratio says speed 0. But the goal is **higher** than the
   shooter, so even a shot from right up close needs a fair amount of speed just to get the ball
   up there. The real line doesn't go through zero.
2. **Physics.** For a ball launched at a fixed angle, the distance it travels grows roughly with
   the **square** of its speed. So twice as far needs about 1.4 times the speed, not 2 times.
   Wheel slip and the ball squishing make it messier still.

So test it, don't assume it.

### The experiment

Use the **straight-line** distance to the tag, `Math.hypot(sideways, distance)`. Print it in
telemetry. For three distances (close, middle, far), use `Shooter Tuning` to find the speed that
scores, then fill this in:

| Straight-line distance (in) | Best speed (tps) | speed ÷ distance |
|-----------------------------|------------------|------------------|
| close:                      |                  |                  |
| middle:                     |                  |                  |
| far:                        |                  |                  |

**If the last column is about the same all three times**, the ratio works. Average them to get `k`:

```java
            double speed = K * straightLine;
```

**If the last column changes** (it most likely gets smaller the farther you go), use a straight
line that doesn't have to go through zero. It's still one multiply and one add:

```java
    // From your close and far measurements:
    //   SLOPE = (farSpeed - closeSpeed) / (farDistance - closeDistance)
    //   BASE  = closeSpeed - SLOPE * closeDistance
    static final double SLOPE = 0;
    static final double BASE = 0;

            double speed = SLOPE * straightLine + BASE;
```

Now **check the middle point**: put its distance into the formula. Is the answer within about 50
tps of the speed you measured? If it is, you're done. If it isn't, the curve bends too much for
one line. Ask your mentor about a **lookup table**: a short list of (distance, speed) pairs, with
the code picking the speed between the two closest ones.

Replace `SHOT_SPEED_TPS` in step 6 with `speed`.

**Checkpoint:** the formula's speed matches what you measured at all three distances, and the
shots go in from each one.

---

## If something goes wrong

- **"not seen" when the tag is right there.** Is `TAG_ID` right? Is Full 3D on? Is the pipeline
  in `TuningConfig.limelightPipeline` the AprilTag one?
- **Numbers jump around a lot.** Too far from the tag, or the tag is at a steep angle. Get closer
  or face it more directly.
- **Drives the wrong way.** Step 5: one direction at a time, flip the sign of the one that's wrong.
- **Wobbles back and forth at the spot and never says "At spot".** Gain too high, or `DONE_IN` /
  `DONE_DEG` too tight. Lower the gain first.
- **Stops just short and sits there.** At small errors the power is too weak to move the robot.
  Raise the gain a little, or loosen `DONE_IN`.
- **Flywheel never ready.** The speed is above what it can hold. See `shooterMaxTps` in
  `TuningConfig`.
