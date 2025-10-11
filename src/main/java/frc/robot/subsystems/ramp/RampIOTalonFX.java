package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.robot.constants.Constants;

public class RampIOTalonFX implements RampIO {
    private final TalonFX roller;
    private static CANrange innerRampSensor;
    private static CANrange outerRampSensor;

    public RampIOTalonFX() {
        roller = new TalonFX(RampConfigTalonFXReal.rollerID, Constants.rampCANBus);
        innerRampSensor = new CANrange(RampConfigTalonFXReal.innerRampSensorID, Constants.rampCANBus);
        outerRampSensor = new CANrange(RampConfigTalonFXReal.outerRampSensorID, Constants.rampCANBus);
        applyConfigs();
    }

    @Override
    public void updateInputs(RampIOInputs inputs) {
        inputs.rollerVelocityRPM = roller.getVelocity().getValueAsDouble();
        inputs.rollerVoltage = roller.getMotorVoltage().getValueAsDouble();
        inputs.outerSensorDetected = outerRampSensor.getIsDetected().getValue();
        inputs.innerSensorDetected = innerRampSensor.getIsDetected().getValue();
    }

    public static void applyConfigs() {
        var CANrangeConfig = new CANrangeConfiguration();
        CANrangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANrangeConfig.FovParams.FOVCenterX = 0; // Reset to default
        CANrangeConfig.FovParams.FOVCenterY = 0;
        CANrangeConfig.FovParams.FOVRangeX = 6.75; // Minimum
        CANrangeConfig.FovParams.FOVRangeY = 6.75; // Minimum
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.10;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.02;
        innerRampSensor.getConfigurator().apply(CANrangeConfig);

        CANrangeConfig.FovParams.FOVCenterX = -11.8;
        CANrangeConfig.FovParams.FOVCenterY = -11.8;
        CANrangeConfig.FovParams.FOVRangeX = 27.0;
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.37;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.05;
        outerRampSensor.getConfigurator().apply(CANrangeConfig);
    }

    @Override
    public void setRollerDuty(double power) {
        roller.setControl(new DutyCycleOut(power));
    }
}
