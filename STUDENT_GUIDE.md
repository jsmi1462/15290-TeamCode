# Student guide

You already have the project cloned and Git installed. This guide covers everything after that.

Use the **Terminal** tab at the bottom of Android Studio for the Git commands. It opens in the project folder.

## 1. One-time setup on each computer

Tell Git who you are, so your work has your name on it:

```
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

The first time you push, GitHub will ask you to sign in. Use the account your mentor added to the repo.

Then in Android Studio, wait for the bar at the bottom to say Gradle sync is finished. The first sync takes a few minutes.

## 2. Make your own branch

Never work directly on the shared branch. You will be on `master`. Make a branch for your work:

```
git switch -c yourname-auto
```

You can check which branch you are on with `git branch`. The one with the star is yours.

## 3. The daily loop

Start of every session, bring the team's latest changes from `master` into your branch:

```
git pull origin master
```

Plain `git pull` is not enough. On your own branch it only checks your own branch, so you would never
see updates from the team. Commit your work first (see below) so the pull has nothing to trip over.

Work, then save a snapshot of your work. Do this whenever something works, not just at the end:

```
git add -A
git commit -m "Drive to the scoring spot and relocalize"
```

Write the message so a teammate could tell what changed. "stuff" and "fix" are not messages.

End of every session, back up your work to GitHub:

```
git push
```

The first push on a new branch will print a command starting with `git push --set-upstream`. Copy it, run it, and you are done.

## 4. Running on the robot

1. Connect your computer to the Control Hub's Wi-Fi.
2. In Terminal: `adb connect 192.168.43.1:5555`
3. Choose the Control Hub as the device at the top of Android Studio, and press the green Run button.
4. On the Driver Station phone, pick your OpMode from the list and press INIT, then play.

If the Hub is missing from the device list, plug it in with USB-C once, then try again.

## 5. Where your work goes

Everything you write is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/team/`.
In the `pedro` folder, the only file you edit is `Constants.java`. That is where your tuned numbers
go (see Tuning below). Leave `Tuning.java` and the `procedures` folder alone.

See `team/README.md` for what each file is. Setting up the Limelight camera is also yours: follow
`LIMELIGHT_GUIDE.md`.

### Draw your path first

Open https://visualizer.pedropathing.com and drag points to lay out your route. It shows the same
field and the same coordinates the robot uses, so what you see is what the code gets.

1. Click the `</>` button, then **Java Code**.
2. Set **Export Mode** to **Coordinates Only** and copy the numbers.
3. Put them in `Poses.java` as named spots. If you are on the other alliance, tick **Mirror Horizontally**.

The Visualizer measures from the bottom-left corner of the field, in inches. So do we.

## Tuning (this is yours)

Tuning the robot is your job, and the robot will not follow paths well until you do it.

1. Connect your computer to the Control Hub's Wi-Fi. Your internet is off while you are on it, so
   `git pull` and `git push` will not work until you switch back.
2. Open http://192.168.43.1:10158 in a browser. You will see the tuners: Mecanum, Pinpoint,
   Foresight and Tests. Work through them in that order.
3. Each tuner ends with a block of code. Paste it over the matching block in `Constants.java`.
4. Set `TUNED = true` in `Constants.java` only after you have pasted all of them.
5. Run Tests to check that the robot really drives where you tell it to.

You will do this again whenever the robot changes: new wheels, a moved odometry pod, a lot of weight added.

## 6. When something goes wrong

- **Red error text in the build.** Read the first error, not the last. It names a file and line number.
- **`git pull origin master` says there is a conflict.** Stop. Do not try to fix it. Get a mentor.
- **You changed something and it got worse.** `git status` shows what changed. `git restore <file>`
  throws away your edits to that file and goes back to your last commit. This cannot be undone.
- **The robot does something unexpected.** Hit STOP on the Driver Station first. Then look at the code.
- **The robot only strafes one direction in auto.** On the Pinpoint, +Y is left and -Y is right. A strafe
  loop that always drives with one power sign and only checks `y > something` can never reverse. Drive
  toward the target instead: `Math.signum(targetY - y) * DRIVE_POWER` (see `strafeToward` in `AutoRed`).
  A positive target goes left, a negative target goes right. If it still goes the wrong way, flip the sign.
- **The robot strafes forever and never stops.** The stop check is probably on the wrong side of the target.
  Print `Y (in)` in telemetry and make sure it moves toward your target, not away from it.

## 7. Rules for the team

- Your branch is yours. Push to it as often as you like.
- Nobody pushes to `master` directly. A mentor merges finished, tested work.
- Before competition, a mentor tags the known-good version, and we do not touch it after that.
