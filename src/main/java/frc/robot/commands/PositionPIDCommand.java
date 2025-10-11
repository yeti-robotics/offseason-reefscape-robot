package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.*;

public class PositionPIDCommand extends Command{

    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private final PPHolonomicDriveController mDriveController;

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(Seconds.of(0.04).in(Seconds));

    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();

    private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



    private PositionPIDCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
        mDriveController = new PPHolonomicDriveController(new PIDConstants(5.5, 0.0, 0.1, 0.0), new PIDConstants(5, 0, 0));
    }

    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0,0,0)));
            swerve.setControl(new SwerveRequest.SwerveDriveBrake());
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        mSwerve.setControl(robotSpeeds.withSpeeds(
                mDriveController.calculateRobotRelativeSpeeds(
                        mSwerve.getState().Pose, goalState
                ))
        );

        xErrLogger.accept(mSwerve.getState().Pose.getX() - goalPose.getX());
        yErrLogger.accept(mSwerve.getState().Pose.getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);

        var rotation = MathUtil.isNear(
                0.0,
                diff.getRotation().getRotations(),
                Rotation2d.fromDegrees(3.0).getRotations(),
                0.0,
                1.0
        );

        var position = diff.getTranslation().getNorm() < Centimeter.of(1.5).in(Meters);

        var speed = Math.sqrt(Math.pow(mSwerve.getState().Speeds.vx, 2) + Math.pow(mSwerve.getState().Speeds.vy, 2)) < InchesPerSecond.of(2).in(MetersPerSecond);

        // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);

        return endTriggerDebouncer.calculate(
                rotation && position && speed
        );
    }
}