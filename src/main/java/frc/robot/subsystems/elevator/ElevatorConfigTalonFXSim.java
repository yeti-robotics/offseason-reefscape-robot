package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElevatorConfigTalonFXSim {
    // Sim Configs
    private static final Slot1Configs SLOT_1_CONFIGS = new Slot1Configs()
            .withKP(4)
            .withKI(0)
            .withKD(48)
            .withKG(0)
            .withKV(0)
            .withKA(1)
            .withKS(0.5)
            .withGravityType(GravityTypeValue.Elevator_Static);
}
