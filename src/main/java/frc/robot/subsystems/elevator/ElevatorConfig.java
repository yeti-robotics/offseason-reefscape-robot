package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 0;
    static final int secondaryElevatorMotorID = 0;
    static final double gearRatio = 0;

    private static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs();
    private static final TalonFXConfiguration primaryTalonFXConfigs =
            new TalonFXConfiguration();
    private static final TalonFXConfiguration secondaryTalonFXConfigs =
            new TalonFXConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
