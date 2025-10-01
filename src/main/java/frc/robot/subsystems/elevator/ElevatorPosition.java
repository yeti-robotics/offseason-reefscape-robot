package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition {
    BOTTOM(0.0),
    L1(0.0),
    L2(0.0),
    L3(0.0),
    L4(0.0),
    HP_INAKE(0.0),
    BARGE(0.0),
    HIGH_ALGAE(0.0),
    LOW_ALGAE(0.0),
    PROCESSOR(0.0);

    private final Angle height;

    ElevatorPosition(double height) {
        this.height = Units.Rotations.of(height);
    }

    public Angle getHeight() {
        return height;
    }
}
