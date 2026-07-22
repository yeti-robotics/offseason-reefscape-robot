package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoreMech.ScoreMechSubsystem;

public class AutoCommands {
    private final ElevatorSubsystem elevator;
    private final CommandSwerveDrivetrain drivetrain;
    private final ScoreMechSubsystem score;

    public AutoCommands(ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, ScoreMechSubsystem score) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.score = score;
    }

    public Command orca1LeftCoral() {
        return Commands.none();
    }

    public Command driveForward() {
        return drivetrain.runOnce(() -> new SwerveRequest.FieldCentric()
                .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage));
    }
}
