package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class RampIOTalonFX implements RampIO {
    private final TalonFX roller;
    private final CANrange innerRampSensor;
    private final CANrange outerRampSensor;

    public RampIOTalonFX() {
        roller = new TalonFX(RampConfigTalonFXReal.rollerID, Constants.motorCANBus);
        innerRampSensor = new CANrange(RampConfigTalonFXReal.innerRampSensorID, Constants.motorCANBus);
        outerRampSensor = new CANrange(RampConfigTalonFXReal.outerRampSensorID, Constants.motorCANBus);
    }

    @Override
    public void updateInputs(RampIOInputs inputs) {
        inputs.rollerVelocityRPM = roller.getVelocity().getValueAsDouble();
        inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
        inputs.outerSensorDetected = outerRampSensor.getIsDetected().getValue();
        inputs.innerSensorDetected = innerRampSensor.getIsDetected().getValue();
    }

    @Override
    public void setRollerVoltage(double volts) {
        roller.setControl(new DutyCycleOut(volts));
    }
}
