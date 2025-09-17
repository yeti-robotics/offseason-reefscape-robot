// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final CANBus motorCANBus = CANBus.systemCore(1);

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final CANBus drivetrainCANBus = new CANBus(TunerConstants.kCANBus.getName(), "./logs/example.hoot");
    public static final CANBus motorCANBus = CANBus.systemCore(1);
    public static final int PRIMARY_CONTROLLER_PORT = 0;

    // Front Camera Red
    public static final Transform3d frontCamTrans = new Transform3d(
            new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0), Units.inchesToMeters(9)),
            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(0)));

    // Rear Camera Blue
    public static final Transform3d rearCamTrans = new Transform3d(
            new Translation3d(Units.inchesToMeters(-3), Units.inchesToMeters(8), Units.inchesToMeters(38)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(160)));
    public static final CANBus motorCANBus = CANBus.systemCore(1);
}
