package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

public class RampIOTalonFX implements RampIO {
    private final TalonFX Roller = new TalonFX(0, Constants.motorCANBus);
    private final CANrange RampSensor = new CANrange(1, Constants.motorCANBus);
    private final CANrange RampSensorTwo = new CANrange(2, Constants.motorCANBus);

    public RampIOTalonFX() {}

    @Override
    public void updateInputs(RampIOInputs inputs) {
        inputs.rollerVelocityRPM = Roller.getVelocity().getValueAsDouble();
        inputs.rollerVoltage = Roller.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setRollerVoltage(double volts) {
        Roller.setVoltage(0.1);
    }
}
