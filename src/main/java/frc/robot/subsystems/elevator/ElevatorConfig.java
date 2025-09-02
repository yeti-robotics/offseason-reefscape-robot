package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;

class ElevatorConfig {
    //Placeholder IDs for now
    static final int primaryElevatorMotorID = 0;
    static final int secondaryElevatorMotorID = 0;
    static final int canRangeID = 0;

    static final double gearRatio = 18/1;

    private static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs();
    private static final Slot1Configs SLOT_1_CONFIGS =
            new Slot1Configs();

    private static final TalonFXConfiguration primaryTalonFXConfigs =
            new TalonFXConfiguration();
    private static final TalonFXConfiguration secondaryTalonFXConfigs =
            new TalonFXConfiguration();

    private static final CANrangeConfiguration canRangeElevatorConfigs =
            new CANrangeConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
