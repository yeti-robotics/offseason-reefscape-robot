package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.motorCANBus;

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
        primaryElevatorMotor = new TalonFX(ElevatorConfigReal.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigReal.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigReal.canRangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = secondaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
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
