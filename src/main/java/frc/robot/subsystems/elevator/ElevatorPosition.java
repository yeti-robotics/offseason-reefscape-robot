package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition {
    BOTTOM(0.0);

    private final Angle height;

    ElevatorPosition(double height) {
        this.height = Units.Rotations.of(height);
    }

    public Angle getHeight() {
        return height;
    }
}
