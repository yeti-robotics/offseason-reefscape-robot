package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.motorCANBus;
import static frc.robot.subsystems.elevator.ElevatorConfigReal.primaryTalonFXConfigs;
import static frc.robot.subsystems.elevator.ElevatorConfigReal.secondaryTalonFXConfigs;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorIOSim implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    public ElevatorIOSim() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigSim.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigSim.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigSim.canRangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
        primaryElevatorMotor.getConfigurator().apply(ElevatorConfigSim.primaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfigReal.primaryElevatorMotorID, true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = secondaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
        inputs.targetPositionRotation = primaryElevatorMotor.getClosedLoopReference().getValueAsDouble();
    }

    @Override
    public void moveToPosition(double position) {
        primaryElevatorMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }

    @Override
    public void zeroPosition() {
        primaryElevatorMotor.setPosition(0);
    }
}
