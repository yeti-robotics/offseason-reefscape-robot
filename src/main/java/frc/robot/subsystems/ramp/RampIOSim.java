package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

public class RampIOSim implements RampIO {
    private final TalonFX roller;
    private final CANrange innerRampSensor;
    private final CANrange outerRampSensor;

    public RampIOSim() {
        roller = new TalonFX(0, Constants.motorCANBus);
        innerRampSensor = new CANrange(3, Constants.motorCANBus);
        outerRampSensor = new CANrange(4, Constants.motorCANBus);
    }

    @Override
    public void updateInputs(RampIOInputs inputs) {
        RampIO.super.updateInputs(inputs);
    }

    @Override
    public void setRollerVoltage(double volts) {
        RampIO.super.setRollerVoltage(volts);
    }
}
