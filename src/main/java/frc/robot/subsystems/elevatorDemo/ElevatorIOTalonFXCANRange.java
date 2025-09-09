package frc.robot.subsystems.elevatorDemo;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

public class ElevatorIOTalonFXCANRange implements ElevatorIO {
    private final TalonFX elevatorMotor;
    private final CANrange bottomSensor;

    public ElevatorIOTalonFXCANRange() {
        elevatorMotor = new TalonFX(44, Constants.motorCANBus);
        bottomSensor = new CANrange(45, Constants.motorCANBus);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = elevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = elevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = bottomSensor.getIsDetected().getValue();
    }

    @Override
    public void moveToPosition(double position) {
        elevatorMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }
}
