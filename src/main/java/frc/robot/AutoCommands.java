package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ReefAlignPPOTF;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoreMech.ScoreMechSubsystem;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;

public class AutoCommands {
    private final ElevatorSubsystem elevator;
    private final ReefAlignPPOTF reefAlignPPOTF;
    private final CommandSwerveDrivetrain drivetrain;
    private final ScoreMechSubsystem score;

    public AutoCommands(
            ElevatorSubsystem elevator,
            ReefAlignPPOTF reefAlignPPOTF,
            CommandSwerveDrivetrain drivetrain,
            ScoreMechSubsystem score) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.reefAlignPPOTF = reefAlignPPOTF;
        this.score = score;
    }

    public Command orca1LeftCoral() {
        Optional<PathPlannerPath> lineToIJ = PathPlannerUtils.loadPathByName("lineToIJ");

        return lineToIJ.isEmpty()
                ? drivetrain.runOnce(() -> new SwerveRequest.FieldCentric()
                .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage))
                : AutoBuilder.followPath(lineToIJ.get()).withTimeout(2)
                        .andThen(reefAlignPPOTF.reefAlign(ReefAlignPPOTF.Branch.LEFT))
                        .andThen(elevator.moveToPosition(ElevatorPosition.L4.getHeight()))
                        .andThen(elevator.moveToPosition(ElevatorPosition.L4_UP.getHeight())
                                .alongWith(score.spinManual(0.5).withTimeout(3)))
                        .andThen(elevator.moveToPosition(ElevatorPosition.BOTTOM.getHeight()));
    }
}
