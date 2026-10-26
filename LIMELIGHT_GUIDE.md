# Limelight setup guide

Setting up the Limelight is your job. This guide tells you what to do and how to know it worked.
It does not do it for you.

Why bother: odometry drifts as the robot drives. The Limelight sees AprilTags on the field and
works out where the robot really is. `LimelightHelper.relocalize(follower)` uses that to snap the
robot's position back to the truth. The code for that is already written. The setup is yours.

The official instructions are at https://docs.limelightvision.io. If anything below disagrees with
them, trust Limelight's docs and tell your mentor so we can fix this page.

## Before you start

- A Limelight 3A, a USB-C to USB-A cable, and a laptop.
- The Control Hub, powered on.
- A tape measure.
- About an hour. Do not rush the last two steps.

## 1. Plug it in and wait

1. For setup, connect the Limelight to your laptop with the USB-C cable. (A mentor can show you
   the port.)
2. Wait 15 to 20 seconds. A green status light means it has started.

## 2. Open the Limelight web page

Open http://limelight.local:5801 in your browser. If that does not load, open the **Limelight
Hardware Manager** app instead.

You will see tabs for settings, pipelines, camera, and a 3D view. Look at each one before you
change anything.

## 3. Check the firmware

Ask yourself: is this the newest Limelight OS? The Hardware Manager's **Flash OS** tab is where you
update it. Updating is only needed if it is out of date. Read Limelight's docs on flashing before you
press anything, because it needs a special button hold while plugging in.

## 4. Settings tab

1. Enter our team number.
2. Press **Restart Vision Client**.

## 5. Make the AprilTag pipeline work

1. Use the AprilTag pipeline (pipeline 0 is the default AprilTag one).
2. In its **Advanced** settings, turn on **Full 3D**. Without it you get no field position.
3. Load the AprilTag field map for **this season's game**. A map for the wrong game puts the tags in
   the wrong places and everything after this will be wrong. Find where it is set and confirm it.

You should now be able to hold a tag in front of the camera and see it detected.

## 6. Tell the Limelight where the camera sits

The Limelight needs to know where the camera is on the robot, measured from the **center of the
robot's footprint**, and which way it points.

1. Measure forward, sideways and up distances from the robot's center to the camera lens.
2. Measure which way it points: straight ahead, tilted up, turned to a side.
3. Enter them on the web page. Check which units it asks for and which direction is positive.

This step decides how accurate your position is. A wrong measurement here gives a position that is
wrong by the same amount every time.

## 7. Check it on the 3D view

Point the robot at a tag. In the 3D visualization tab, does the robot appear where it really is
relative to the tag? Move the robot and watch it follow. If it does not, go back to step 6.

## 8. Plug it into the Control Hub

1. Plug the Limelight into the Control Hub's **blue** USB 3.0 port.
2. Open the Robot Configuration on the Driver Station phone, pick **Configure Robot**, and scan.
3. The Limelight shows up as an **Ethernet Device**. Rename it to exactly `limelight`, all lower
   case. Our code looks for that name. Save the configuration and make it the active one.

## 9. Check that the code and the camera agree

This is the step most teams skip, and it is where most problems hide.

Our code uses a field frame with its origin at the field corner, in inches. The Limelight reports
from the field center, in meters. `LimelightHelper` converts between them. Prove the conversion is
right:

1. Put the robot at a spot you can measure on the field, such as a known distance from two walls.
2. Run a short OpMode that sets the robot's pose to that spot, calls `relocalize`, and prints the
   pose before and after. You write this OpMode.
3. Compare what it prints with where the robot really is.

Questions to answer from what you see:

- Do x and y match reality, or are they swapped?
- Is the position mirrored, left to right or front to back?
- Is it off by a fixed amount in one direction? What could cause that?
- Heading 0 means something specific to the Limelight and something specific to Pedro. Do they
  agree?

When x, y and heading all match, you are done. Write down what you changed so the next person
knows.

## If something goes wrong

- **The web page will not load.** Check the cable, wait a full 20 seconds, try the Hardware
  Manager.
- **No position at all.** Is Full 3D on? Is the correct field map loaded? Is a tag in view?
- **Position is wrong by a steady amount.** Re-measure the camera position from step 6.
- **`relocalize` returns false.** No usable tag was in view, or the reading was too old. Look at
  `LimelightHelper` to see which.
- **Nothing makes sense.** Stop and explain the problem out loud to a teammate. You can also ask
  Claude Code to help you understand it, but it will explain and ask questions, not do the setup.
