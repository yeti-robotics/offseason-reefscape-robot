package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

class ElevatorConfigReal {
    static final int primaryElevatorMotorID = 14;
    static final int secondaryElevatorMotorID = 15;
    static final int canRangeID = 16;

    static final double gearRatio = 0;

    private static final Slot0Configs SLOT_0_CONFIGS = new Slot0Configs();

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration();
    static final TalonFXConfiguration secondaryTalonFXConfigs = new TalonFXConfiguration();

    static final CANrangeConfiguration canRangeElevatorConfigs = new CANrangeConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
