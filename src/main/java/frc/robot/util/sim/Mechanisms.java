package frc.robot.util.sim;

import org.wpilib.epilogue.Logged;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

@Logged
public class Mechanisms {
    public Mechanism2d elevatorMech;
    private final MechanismLigament2d liftLigament;

    public Mechanisms() {
        elevatorMech = new Mechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));

        liftLigament = elevatorMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("lift", Units.feetToMeters(3), 90, 6, new Color8Bit(Color.kRed)));
        elevatorMech
                .getRoot("startPoint", Units.inchesToMeters(30), Units.inchesToMeters(4))
                .append(new MechanismLigament2d("bottom", Units.feetToMeters(3), 0, 6, new Color8Bit(Color.kGreen)));
    }

    public void updateElevatorMech(double elevatorPos) {
        liftLigament.setLength(Units.inchesToMeters((elevatorPos * 6) + 1));
        SmartDashboard.putData("Mechanisms/CoralManipulator", elevatorMech);
    }

    public void publishComponentPoses(double elevatorPos, boolean useRealPoses) {
        double elevatorStageHeight = Units.inchesToMeters(elevatorPos * 8.6);
        double carriageHeight = Units.inchesToMeters(elevatorPos * 15);

        Logger.recordOutput(
                "ComponentPoses/" + (useRealPoses ? "Real" : "Target"),
                new Pose3d(
                        Units.inchesToMeters(-8),
                        0.0,
                        Units.inchesToMeters(2.625) + elevatorStageHeight,
                        Rotation3d.kZero),
                new Pose3d(
                        Units.inchesToMeters(-4.13),
                        0,
                        Units.inchesToMeters(10.22) + carriageHeight,
                        Rotation3d.kZero));
    }
}
