// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final CANBus drivetrainCANBus = new CANBus(TunerConstants.kCANBus.getName(), "./logs/example.hoot");
    public static final int PRIMARY_CONTROLLER_PORT = 0;

    //reef & scoring cam
    public static final Transform3d camTrans1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0), Units.inchesToMeters(7)),
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(0)));

    //left elevator cam
    public static final Transform3d camTrans2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(-12), Units.inchesToMeters(40)),
            new Rotation3d(0, Math.toRadians(10), Math.toRadians(200)));

    //right elevator cam
    public static final Transform3d camTrans3 = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(17), Units.inchesToMeters(40)),
            new Rotation3d(0, Math.toRadians(5), Math.toRadians(90)));
}
