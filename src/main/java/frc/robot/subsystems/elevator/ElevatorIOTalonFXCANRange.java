package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOTalonFXCANRange implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    public ElevatorIOTalonFXCANRange() {
        primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfig.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfig.canRangeID, motorCANBus);
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
