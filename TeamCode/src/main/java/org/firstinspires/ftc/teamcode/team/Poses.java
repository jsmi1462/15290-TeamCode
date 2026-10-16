package org.firstinspires.ftc.teamcode.team;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Named spots on the field.
 *
 * The frame: origin at the bottom-left CORNER of the field, units in INCHES, heading in DEGREES.
 * This is the same frame the Pedro Visualizer uses, so you can copy its numbers straight in.
 *
 * START and SCORE are examples with made-up numbers. Replace them with real ones, then add
 * the rest of the spots your auto needs.
 */
public final class Poses {
    private Poses() {}

    private static final PoseFactory pose = PoseFactory.degrees();

    //                                         x (in)  y (in)  heading (deg)
    public static final Pose START = pose.of(12, 36, 0);
    public static final Pose SCORE = pose.of(36, 60, 45);

    // TODO: add the rest of your spots
}
