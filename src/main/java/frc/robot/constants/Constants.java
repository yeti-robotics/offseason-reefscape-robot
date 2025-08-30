// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int PRIMARY_CONTROLLER_PORT = 0;

    public static final Transform3d camTrans1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0), Units.inchesToMeters(7)),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(0)));

    public static final Transform3d camTrans2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(-12), Units.inchesToMeters(40)),
            new Rotation3d(0, Math.toRadians(30), Math.toRadians(200)));

    public static final Transform3d camTrans3 = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(17), Units.inchesToMeters(40)),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(90)));
}
