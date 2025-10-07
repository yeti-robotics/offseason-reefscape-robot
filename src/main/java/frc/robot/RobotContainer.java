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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.util.sim.Mechanisms;
import frc.robot.util.sim.vision.AprilTagSimulator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain;
    private final ScoreMechSubsystem score;
    private final ElevatorSubsystem elevator;

    private final RampSubsystem ramp;

    private boolean algaeMode = false;

    // Vision
    //    public final PhotonAprilTagSystem frontCam;
    //    public final PhotonAprilTagSystem rearCam;
    //    private final AprilTagSubsystem[] aprilTagSubsystems;
    final Mechanisms mechanisms;

    AprilTagSimulator aprilTagCamSim = new AprilTagSimulator();

    //    public void updateVision() {
    //        for (AprilTagSubsystem aprilTagSubsystem : aprilTagSubsystems) {
    //            List<AprilTagPose> aprilTagPoseOpt = aprilTagSubsystem.getEstimatedPose();
    //
    //            if (!aprilTagPoseOpt.isEmpty() && !drive.isMotionBlur()) {
    //                for (AprilTagPose pose : aprilTagPoseOpt) {
    //                    if (pose.numTags() > 0) {
    //                        drive.addVisionMeasurement(
    //                                pose.estimatedRobotPose(), pose.timestamp(), pose.standardDeviations());
    //                    }
    //                }
    //            }
    //        }
    //    }

    //    public void updateVisionSim() {
    //        aprilTagCamSim.update(drive.getPose());
    //        Pose3d frontCameraPose = new Pose3d(drive.getPose())
    //                .transformBy(new Transform3d(
    //                        Constants.frontCamTrans.getX(),
    //                        Constants.frontCamTrans.getY(),
    //                        Constants.frontCamTrans.getZ(),
    //                        Constants.frontCamTrans.getRotation()));
    //
    //        Pose3d rearCameraPose = new Pose3d(drive.getPose())
    //                .transformBy(new Transform3d(
    //                        Constants.rearCamTrans.getX(),
    //                        Constants.rearCamTrans.getY(),
    //                        Constants.rearCamTrans.getZ(),
    //                        Constants.rearCamTrans.getRotation()));
    //
    //        Logger.recordOutput("Front Cam Transform", frontCameraPose);
    //        Logger.recordOutput("Rear Cam Transform", rearCameraPose);
    //    }

    //    private final SwerveRequest.FieldCentric driveForward = new SwerveRequest.FieldCentric()
    //            .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
    //            .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1)
    //            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

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

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = TunerConstants.createDrivetrain();
                score = new ScoreMechSubsystem(new ScoreMechIOSim());
                elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());

                ramp = new RampSubsystem(new RampIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = TunerConstants.createDrivetrain();
                score = new ScoreMechSubsystem(new ScoreMechIO() {});
                elevator = new ElevatorSubsystem(new ElevatorIO() {});

                ramp = new RampSubsystem(new RampIO() {});

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        //        frontCam = new PhotonAprilTagSystem("FrontCam", Constants.frontCamTrans, drive);
        //        rearCam = new PhotonAprilTagSystem("RearCam", Constants.rearCamTrans, drive);

        mechanisms = new Mechanisms();

        //        AprilTagCamSim simCam1 = AprilTagCamSimBuilder.newCamera()
        //                .withCameraName("FrontCam")
        //                .withTransform(Constants.frontCamTrans)
        //                .build();
        //        aprilTagCamSim.addCamera(simCam1);
        //        frontCam.setCamera(simCam1.getCam());
        //
        //        AprilTagCamSim simCam2 = AprilTagCamSimBuilder.newCamera()
        //                .withCameraName("RearCam")
        //                .withTransform(Constants.rearCamTrans)
        //                .build();
        //        aprilTagCamSim.addCamera(simCam2);
        //        rearCam.setCamera(simCam2.getCam());
        //
        //        aprilTagSubsystems = new AprilTagSubsystem[] {frontCam, rearCam};

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

        controller.rightTrigger().whileTrue(score.spinManual(0.2));
        controller
                .leftTrigger()
                .whileTrue(elevator.moveToPosition(ElevatorPosition.HP_INAKE.getHeight())
                        .andThen(ramp.setRoller(0.5))
                        .andThen(score.spinManual(0.5))); // check voltage?

        controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller.povRight().onTrue(new InstantCommand(() -> algaeMode = !algaeMode));
        controller.povLeft().onTrue(score.spinUntilCoralSafe());

        controller.y().onTrue(elevator.moveToPosition(ElevatorPosition.L4.getHeight()));
        controller
                .x()
                .onTrue(Commands.either(
                        elevator.moveToPosition(ElevatorPosition.HIGH_ALGAE.getHeight()),
                        elevator.moveToPosition(ElevatorPosition.L3.getHeight()),
                        () -> algaeMode));
        controller
                .b()
                .onTrue(Commands.either(
                        elevator.moveToPosition(ElevatorPosition.LOW_ALGAE.getHeight()),
                        elevator.moveToPosition(ElevatorPosition.L2.getHeight()),
                        () -> algaeMode));
        controller
                .a()
                .onTrue(Commands.either(
                        elevator.moveToPosition(ElevatorPosition.PROCESSOR.getHeight()),
                        elevator.moveToPosition(ElevatorPosition.L1.getHeight()),
                        () -> algaeMode));
    }

    private void configureTriggerBindings() {
        // Trigger for coral detection in ramp - will automatically set coral position for handoff
        new Trigger(ramp::outerRollerDetection).whileTrue(ramp.setRoller(0.5));
        new Trigger(ramp::innerRollerDetection)
                .onTrue(score.spinUntilCoralSafe().andThen(ramp.setRoller(0)));
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
