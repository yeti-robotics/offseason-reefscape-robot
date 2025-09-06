package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

class ElevatorConfig {
    //Placeholder IDs for now
    static final int primaryElevatorMotorID = 0;
    static final int secondaryElevatorMotorID = 0;
    static final int canRangeID = 0;

    static final double gearRatio = 18/1;

    //Real Configs
    private static final Slot0Configs SLOT_0_CONFIGS =
            new Slot0Configs();
    //Sim Configs
    private static final Slot1Configs SLOT_1_CONFIGS = new Slot1Configs()
            .withKP(4)
            .withKI(0)
            .withKD(48)
            .withKG(0)
            .withKV(0)
            .withKA(1)
            .withKS(0.5)
            .withGravityType(GravityTypeValue.Elevator_Static);

    static final TalonFXConfiguration primaryTalonFXConfigs =
            new TalonFXConfiguration();
    static final TalonFXConfiguration secondaryTalonFXConfigs =
            new TalonFXConfiguration();

    static final CANrangeConfiguration canRangeElevatorConfigs =
            new CANrangeConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
