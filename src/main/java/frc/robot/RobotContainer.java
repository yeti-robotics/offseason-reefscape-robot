// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeAlignPPOTF;
import frc.robot.commands.AutoNamedCommands;
import frc.robot.commands.ReefAlignPPOTF;
import frc.robot.constants.Constants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.ramp.*;
import frc.robot.subsystems.scoreMech.ScoreMechIO;
import frc.robot.subsystems.scoreMech.ScoreMechIOSim;
import frc.robot.subsystems.scoreMech.ScoreMechIOTalonFXCANrange;
import frc.robot.subsystems.scoreMech.ScoreMechSubsystem;
import frc.robot.subsystems.vision.PhotonAprilTagSystem;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagCamSim;
import frc.robot.util.sim.vision.AprilTagCamSimBuilder;
import frc.robot.util.sim.vision.AprilTagSimulator;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final CommandSwerveDrivetrain drivetrain;
    private final ScoreMechSubsystem score;
    private final ElevatorSubsystem elevator;

    private final RampSubsystem ramp;

    public boolean algaeMode = false;

    // Vision
    public final PhotonAprilTagSystem leftFrontCam;
    public final PhotonAprilTagSystem rightFrontCam;
    public final PhotonAprilTagSystem rearCam;
    private final AprilTagSubsystem[] aprilTagSubsystems;
    private final ReefAlignPPOTF reefAlignPPOTF;
    private final AlgaeAlignPPOTF algaeAlignPPOTF;

    final Mechanisms mechanisms;

    AprilTagSimulator aprilTagCamSim = new AprilTagSimulator();

    public final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/Pose", Pose2d.struct)
            .publish();

    public void updateVision() {
        for (AprilTagSubsystem aprilTagSubsystem : aprilTagSubsystems) {
            List<AprilTagPose> aprilTagPoseOpt = aprilTagSubsystem.getEstimatedPose();

            if (!aprilTagPoseOpt.isEmpty() && !drivetrain.isMotionBlur()) {
                for (AprilTagPose pose : aprilTagPoseOpt) {
                    if (pose.numTags() > 0) {
                        drivetrain.addVisionMeasurement(
                                pose.estimatedRobotPose(), pose.timestamp(), pose.standardDeviations());
                    }
                }
            }
        }

        posePublisher.set(drivetrain.getState().Pose);
    }

    public void updateVisionSim() {
        aprilTagCamSim.update(drivetrain.getState().Pose);
        Pose3d leftFrontCameraPose = new Pose3d(drivetrain.getState().Pose)
                .transformBy(new Transform3d(
                        Constants.leftFrontCamTrans.getX(),
                        Constants.leftFrontCamTrans.getY(),
                        Constants.leftFrontCamTrans.getZ(),
                        Constants.leftFrontCamTrans.getRotation()));

        Pose3d rearCameraPose = new Pose3d(drivetrain.getState().Pose)
                .transformBy(new Transform3d(
                        Constants.rearCamTrans.getX(),
                        Constants.rearCamTrans.getY(),
                        Constants.rearCamTrans.getZ(),
                        Constants.rearCamTrans.getRotation()));

        Pose3d rightFrontCameraPose = new Pose3d(drivetrain.getState().Pose)
                .transformBy(
                        new Transform3d(
                                Constants.rightFrontCamTrans.getX(),
                                Constants.rightFrontCamTrans.getY(),
                                Constants.rightFrontCamTrans.getZ(),
                                Constants.rightFrontCamTrans.getRotation()
                        )
                );

        Logger.recordOutput("Left Front Cam Transform", leftFrontCameraPose);
        Logger.recordOutput("Rear Cam Transform", rearCameraPose);
        Logger.recordOutput("Right Front Cam Transform", rightFrontCameraPose);

        aprilTagCamSim.update(drivetrain.getState().Pose);
    }

    private final SwerveRequest.FieldCentric driveForward = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Controller
    private final CommandXboxController controller = new CommandXboxController(Constants.PRIMARY_CONTROLLER_PORT);

    // Random ahh swerve request
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = TunerConstants.createDrivetrain();
                score = new ScoreMechSubsystem(new ScoreMechIOTalonFXCANrange());

                elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
                ramp = new RampSubsystem(new RampIOTalonFX());

                leftFrontCam = new PhotonAprilTagSystem("Left Front Camera", Constants.leftFrontCamTrans, drivetrain);
                rightFrontCam = new PhotonAprilTagSystem("Right Front Camera", Constants.rightFrontCamTrans, drivetrain);
                rearCam = new PhotonAprilTagSystem("Rear Camera", Constants.rearCamTrans, drivetrain);

                reefAlignPPOTF = new ReefAlignPPOTF(drivetrain, leftFrontCam, rightFrontCam, rearCam);
                algaeAlignPPOTF = new AlgaeAlignPPOTF(drivetrain, leftFrontCam, rearCam);

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = TunerConstants.createDrivetrain();
                score = new ScoreMechSubsystem(new ScoreMechIOSim());
                elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());

                ramp = new RampSubsystem(new RampIOSim());

                leftFrontCam = new PhotonAprilTagSystem("Front Camera", Constants.leftFrontCamTrans, drivetrain);
                rightFrontCam = new PhotonAprilTagSystem("Right Front Camera", Constants.rightFrontCamTrans, drivetrain);
                rearCam = new PhotonAprilTagSystem("Rear Camera", Constants.rearCamTrans, drivetrain);

                reefAlignPPOTF = new ReefAlignPPOTF(drivetrain, leftFrontCam, rightFrontCam, rearCam);
                algaeAlignPPOTF = new AlgaeAlignPPOTF(drivetrain, leftFrontCam, rearCam);

                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = TunerConstants.createDrivetrain();
                score = new ScoreMechSubsystem(new ScoreMechIO() {});
                elevator = new ElevatorSubsystem(new ElevatorIO() {});

                ramp = new RampSubsystem(new RampIO() {});

                leftFrontCam = new PhotonAprilTagSystem("Front Camera", Constants.leftFrontCamTrans, drivetrain);
                rightFrontCam = new PhotonAprilTagSystem("Right Front Camera", Constants.rightFrontCamTrans, drivetrain);
                rearCam = new PhotonAprilTagSystem("Rear Camera", Constants.rearCamTrans, drivetrain);

                reefAlignPPOTF = new ReefAlignPPOTF(drivetrain, leftFrontCam, rightFrontCam, rearCam);
                algaeAlignPPOTF = new AlgaeAlignPPOTF(drivetrain, leftFrontCam, rearCam);

                break;
        }

        // Set up auto routines
        var namedCommands = new AutoNamedCommands(score, elevator, reefAlignPPOTF);
        namedCommands.registerCommands();

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        mechanisms = new Mechanisms();

        AprilTagCamSim simCam1 = AprilTagCamSimBuilder.newCamera()
                .withCameraName("Front Camera")
                .withTransform(Constants.leftFrontCamTrans)
                .build();
        aprilTagCamSim.addCamera(simCam1);
        leftFrontCam.setCamera(simCam1.getCam());

        AprilTagCamSim simCam2 = AprilTagCamSimBuilder.newCamera()
                .withCameraName("Rear Camera")
                .withTransform(Constants.rearCamTrans)
                .build();
        aprilTagCamSim.addCamera(simCam2);
        rearCam.setCamera(simCam2.getCam());

        AprilTagCamSim simCam3 = AprilTagCamSimBuilder.newCamera()
                .withCameraName("Right Front Camera")
                .withTransform(Constants.rightFrontCamTrans)
                .build();
        aprilTagCamSim.addCamera(simCam3);
        rightFrontCam.setCamera(simCam3.getCam());

        aprilTagSubsystems = new AprilTagSubsystem[] {leftFrontCam, rightFrontCam, rearCam};

        // Configure the button bindings
        configureButtonBindings();

        // Configure the trigger bindings
        configureTriggerBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                () -> drive.withVelocityX(-controller.getLeftY() * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(-controller.getLeftX() * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withRotationalRate(-controller.getRightX() * TunerConstants.MaFxAngularRate)));

//        controller.rightTrigger().onTrue(score.spinManual(0.5).withTimeout(3));
//        controller
//                .leftTrigger()
//                .onTrue(elevator.moveToPosition(ElevatorPosition.HP_INAKE.getHeight())); // check voltage?
//
//        controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
//        controller.povRight().onTrue(new InstantCommand(() -> algaeMode = !algaeMode));
//        controller.povLeft().whileTrue(ramp.setRoller(-0.1).alongWith(score.spinManual(-0.2)));
//        controller.povUp().whileTrue(ramp.setRoller(0.75).andThen(score.spinManual(0.2)));
//
//        controller.y().onTrue(elevator.moveToPosition(ElevatorPosition.L4.getHeight()));
//        controller
//                .x()
//                .onTrue(Commands.either(
//                        elevator.moveToPosition(ElevatorPosition.HIGH_ALGAE.getHeight()),
//                        elevator.moveToPosition(ElevatorPosition.L3.getHeight()),
//                        () -> algaeMode));
//        controller
//                .b()
//                .onTrue(Commands.either(
//                        elevator.moveToPosition(ElevatorPosition.LOW_ALGAE.getHeight()),
//                        elevator.moveToPosition(ElevatorPosition.L2.getHeight()),
//                        () -> algaeMode));
//        controller
//                .a()
//                .onTrue(Commands.either(
//                        elevator.moveToPosition(ElevatorPosition.PROCESSOR.getHeight()),
//                        elevator.moveToPosition(ElevatorPosition.L1.getHeight()),
//                        () -> algaeMode));
//
//        controller.povDown().onTrue(elevator.moveToPosition(ElevatorPosition.BOTTOM.getHeight()));

        controller.leftBumper().whileTrue(reefAlignPPOTF.reefAlign(ReefAlignPPOTF.Branch.LEFT));
        controller.rightBumper().whileTrue(reefAlignPPOTF.reefAlign(ReefAlignPPOTF.Branch.RIGHT));
    }

    private void configureTriggerBindings() {
        // Trigger for coral detection in ramp - will automatically set coral position for handoff
        new Trigger(ramp::outerRampDetection)
                .or(ramp::innerRampDetection)
                .whileTrue(ramp.setRoller(0.75).alongWith(score.spinManual(0.05)));
        new Trigger(score::innerSensorDetected)
                .and(() -> !score.outerSensorDetected())
                .debounce(0.2)
                .onTrue(score.spinUntilCoralSafe()
                        .andThen(score.spinManual(-0.07).until(score::innerSensorDetected))
                        .onlyIf(() -> elevator.getCurrentPosition() <= 0.05));
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(elevator.getCurrentPosition(), true);
        mechanisms.publishComponentPoses(elevator.getTargetPosition(), false);
        mechanisms.updateElevatorMech(elevator.getCurrentPosition());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
