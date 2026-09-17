package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.rioBus;
import static frc.robot.subsystems.elevator.ElevatorConfig.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorIOTalonFX implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0);

    public ElevatorIOTalonFX() {
        primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID, rioBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID, rioBus);
        canRangeElevator = new CANrange(ElevatorConfig.canRangeID, rioBus);
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
        }
        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, MotorAlignmentValue.Opposed));
        canRangeElevator.getConfigurator().apply(canRangeElevatorConfigs);

        zeroPosition();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = primaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = primaryElevatorMotor.getPosition().getValueAsDouble() < 0.03;
        inputs.targetPositionRotation =
                primaryElevatorMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void moveToPosition(Angle position) {
        primaryElevatorMotor.setControl(magicRequest.withPosition(position));
    }

    @Override
    public void zeroPosition() {
        primaryElevatorMotor.setPosition(0);
    }

    @Override
    public void neutralizeElevator() {
        primaryElevatorMotor.setControl(new NeutralOut());
    }
}
