package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;

public class RampIOSim implements RampIO {
    private final TalonFX roller;
    private final CANrange innerRampSensor;
    private final CANrange outerRampSensor;

    public RampIOSim() {
        roller = new TalonFX(0, Constants.motorCANBus);
        innerRampSensor = new CANrange(3, Constants.motorCANBus);
        outerRampSensor = new CANrange(4, Constants.motorCANBus);
        PhysicsSim.getInstance().addTalonFX(roller);
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
