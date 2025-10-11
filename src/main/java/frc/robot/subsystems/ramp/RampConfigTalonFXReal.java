package frc.robot.subsystems.ramp;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class RampConfigTalonFXReal {
    static final int rollerID = 33;
    static final int innerRampSensorID = 2;
    static final int outerRampSensorID = 0;

    static TalonFXConfiguration rampConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
