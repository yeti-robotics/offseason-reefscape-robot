package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ReefAlignPPOTF {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;

    private static final SwerveRequest.FieldCentricFacingAngle swerveReq = new SwerveRequest.FieldCentricFacingAngle();
    private static final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();

    private static final Transform2d leftBranchTransform = new Transform2d(
            Units.inchesToMeters(6),
            Units.inchesToMeters(-2.5),
            Rotation2d.k180deg); // negative x gets you closer, positive is further
    private static final Transform2d rightBranchTransform = new Transform2d(
            Units.inchesToMeters(6),
            Units.inchesToMeters(2.5),
            Rotation2d.k180deg); // negative y moves you left, positive moves right

    public enum Branch {
        LEFT,
        RIGHT
    }

    private Pose2d reefFaceTargetPose;

    public ReefAlignPPOTF(
            CommandSwerveDrivetrain commandSwerveDrivetrain, AprilTagSubsystem reefCam1, AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam1 = reefCam1;

        swerveReq.HeadingController.setPID(10, 0, 1);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean isRedReef(int id) {
        return id > 5 && id < 12;
    }

    public boolean isBlueReef(int id) {
        return id > 16 && id < 23;
    }

    public boolean isOnReef(int id) {
        return isRedReef(id) || isBlueReef(id);
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        return reefCam1.getBestDetection();
    }

    public Optional<Pose2d> getBranchPoseFromTagID(int id) {
        boolean isRedAllianceReef = isRedReef(id);

        if (!isRedAllianceReef && !isBlueReef(id)) {
            return Optional.empty();
        }

        int branchPoseIndex = id - (isRedAllianceReef ? 7 : 18);
        Pose2d[] reefTargetFaces = isRedAllianceReef ? Reef.redCenterFaces : Reef.blueCenterFaces;

        if (branchPoseIndex > 5) {
            return Optional.empty();
        }

        if (branchPoseIndex == -1) {
            branchPoseIndex = reefTargetFaces.length - 1;
        }

        if (branchPoseIndex < 0) {
            return Optional.empty();
        }

        Pose2d reefTargetPose = reefTargetFaces[branchPoseIndex];

        return Optional.of(reefTargetPose);
    }

    private LinearVelocity getChassisVelocity(ChassisSpeeds chassisSpeeds) {
        return MetersPerSecond.of(new Translation2d(chassisSpeeds.vx, chassisSpeeds.vy).getNorm());
    }

    private Command autoAlign(Branch branch) {
        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            return Commands.runOnce(() -> commandSwerveDrivetrain.setControl(stopReq), commandSwerveDrivetrain);
        }

        int fiducialId = detectionOpt.get().getFiducialID();
        System.out.println("Detection id: " + fiducialId);
        Optional<Pose2d> reefTargetPoseOpt = getBranchPoseFromTagID(fiducialId);

        if (reefTargetPoseOpt.isEmpty()) {
            return Commands.runOnce(() -> commandSwerveDrivetrain.setControl(stopReq), commandSwerveDrivetrain);
        }

        reefFaceTargetPose = reefTargetPoseOpt.get();

        Transform2d branchTransform;

        branchTransform = branch == Branch.LEFT ? leftBranchTransform : rightBranchTransform;

        // Get the target position from the branch pose but keep the current rotation
        Pose2d currentPose = commandSwerveDrivetrain.getState().Pose;
        Pose2d targetPosition = reefFaceTargetPose.transformBy(branchTransform);

        Logger.recordOutput("Target pose", targetPosition);
        // Create a new pose with the target position but current rotation
        Pose2d reefBranchPose = new Pose2d(
                targetPosition.getX(), targetPosition.getY(), targetPosition.getRotation() // Keep the current rotation
                );

        return Commands.defer(
                () -> {
                    SwerveDrivetrain.SwerveDriveState state = commandSwerveDrivetrain.getState();
                    Pose2d drivetrainPose = state.Pose;

                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrainPose, reefBranchPose);

                    PathPlannerPath path = new PathPlannerPath(
                            waypoints,
                            new PathConstraints(
                                    MetersPerSecond.of(1),
                                    MetersPerSecondPerSecond.of(2),
                                    RadiansPerSecond.of(2 * Math.PI),
                                    RadiansPerSecondPerSecond.of(4 * Math.PI)),
                            new IdealStartingState(
                                    getChassisVelocity(state.Speeds),
                                    commandSwerveDrivetrain.getRotation3d().toRotation2d()),
                            new GoalEndState(0.0, reefBranchPose.getRotation()));
                    path.preventFlipping = true;

                    PathPlannerTrajectoryState endState = new PathPlannerTrajectoryState();
                    endState.pose = reefBranchPose;

                    Command positionPIDCommand = PositionPIDCommand.generateCommand(commandSwerveDrivetrain, reefBranchPose, Seconds.of(3));

                    return AutoBuilder.followPath(path)
                            .andThen(positionPIDCommand);
                },
                Set.of(commandSwerveDrivetrain));
    }

    public Command reefAlign(Branch branch) {
        return Commands.defer(() -> autoAlign(branch), Set.of(commandSwerveDrivetrain));
    }
}
