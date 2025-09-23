package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Robot;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 14;
    static final int secondaryElevatorMotorID = 15;
    static final int canRangeID = 16;

    static final double gearRatio = 0;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isSimulation()
            ? new Slot0Configs()
                    .withKP(4)
                    .withKI(0)
                    .withKD(48)
                    .withKG(0)
                    .withKV(0)
                    .withKA(1)
                    .withKS(0.5)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            : new Slot0Configs();

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration().withSlot0(SLOT_0_CONFIGS);
    static final TalonFXConfiguration secondaryTalonFXConfigs = new TalonFXConfiguration();

    static final CANrangeConfiguration canRangeElevatorConfigs = new CANrangeConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
