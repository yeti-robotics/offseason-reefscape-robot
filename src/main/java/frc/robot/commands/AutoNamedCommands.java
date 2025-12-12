package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.scoreMech.ScoreMechSubsystem;

public class AutoNamedCommands {
    private final ScoreMechSubsystem score;
    private final ElevatorSubsystem elevator;

    private final ReefAlignPPOTF reefAlignCommand;

    public AutoNamedCommands(ScoreMechSubsystem score, ElevatorSubsystem elevator, ReefAlignPPOTF reefAlignCommand) {
        this.score = score;
        this.elevator = elevator;
        this.reefAlignCommand = reefAlignCommand;
        registerCommands();
    }

    public void registerCommands() {
        NamedCommands.registerCommand(
                "ReefAlignLeft",
                reefAlignCommand
                        .reefAlign(ReefAlignPPOTF.Branch.LEFT)
                        .withTimeout(3)
                        .asProxy());
        NamedCommands.registerCommand(
                "ReefAlignRight",
                reefAlignCommand
                        .reefAlign(ReefAlignPPOTF.Branch.RIGHT)
                        .withTimeout(3)
                        .asProxy());
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
    }
}
