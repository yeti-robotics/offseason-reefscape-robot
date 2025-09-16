package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFX implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    public ElevatorIOTalonFX() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigTalonFXReal.canRangeID, motorCANBus);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = primaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
    }

    @Override
    public void moveToPosition(double position) {
        primaryElevatorMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }
}
