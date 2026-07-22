package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoreMech.ScoreMechSubsystem;

public class AutoNamedCommands {
    private final ScoreMechSubsystem score;
    private final ElevatorSubsystem elevator;

    public AutoNamedCommands(ScoreMechSubsystem score, ElevatorSubsystem elevator) {
        this.score = score;
        this.elevator = elevator;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand("ReefAlignLeft", Commands.none());
        NamedCommands.registerCommand("ReefAlignRight", Commands.none());
        NamedCommands.registerCommand(
                "L4",
                elevator.moveToPosition(ElevatorPosition.L4.getHeight())
                        .andThen(elevator.moveToPosition(ElevatorPosition.L4_UP.getHeight())
                                .alongWith(score.spinManual(0.5).withTimeout(3))));
        NamedCommands.registerCommand(
                "L3", elevator.moveToPosition(ElevatorPosition.L3.getHeight()).andThen(score.scoreCoral()));
        NamedCommands.registerCommand(
                "L2", elevator.moveToPosition(ElevatorPosition.L2.getHeight()).andThen(score.scoreCoral()));
        NamedCommands.registerCommand(
                "L1", elevator.moveToPosition(ElevatorPosition.L1.getHeight()).andThen(score.scoreCoral()));
        NamedCommands.registerCommand("Stow", elevator.moveToPosition(ElevatorPosition.BOTTOM.getHeight()));
        NamedCommands.registerCommand("ReefAlign", Commands.none());
        NamedCommands.registerCommand("AutoAlignRightBranch", Commands.none());
        NamedCommands.registerCommand("GroundIntake", Commands.none());
        NamedCommands.registerCommand("LollipopIntake", Commands.none());
        NamedCommands.registerCommand("Score4", Commands.none());
    }
}
