package frc.robot.subsystems.elevatorDemo;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Constants;

public class ElevatorIOTalonFXBeamBreak implements ElevatorIO {
    private final TalonFX elevatorMotor;
    private final DigitalInput bottomSensor;

    public ElevatorIOTalonFXBeamBreak() {
        elevatorMotor = new TalonFX(44, Constants.motorCANBus);
        bottomSensor = new DigitalInput(5);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = elevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = elevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = bottomSensor.get();
    }

    @Override
    public void moveToPosition(double position) {
        elevatorMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }
}
