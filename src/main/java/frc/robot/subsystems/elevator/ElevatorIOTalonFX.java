package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.motorCANBus;
import static frc.robot.subsystems.elevator.ElevatorConfigReal.primaryTalonFXConfigs;
import static frc.robot.subsystems.elevator.ElevatorConfigReal.secondaryTalonFXConfigs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorIOTalonFX implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    public ElevatorIOTalonFX() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigReal.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigReal.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigReal.canRangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfigReal.primaryElevatorMotorID, true));

        zeroPosition();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = primaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
        inputs.targetPositionRotation = primaryElevatorMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void moveToPosition(double position) {
        primaryElevatorMotor.setControl(magicRequest);
    }

    @Override
    public void zeroPosition() {
        primaryElevatorMotor.setPosition(0);
    }
}
