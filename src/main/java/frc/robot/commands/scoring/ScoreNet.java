package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;

public class ScoreNet extends SequentialCommandGroup {
    private double start_time = 0.0;

    public ScoreNet(Superstructure superstruct, Intake intake) {
        
        addCommands(
            new SuperToState(superstruct, 1, SuperPreset.NET.getState()),
            (Commands.sequence(
                Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
                intake.ingest(() -> 10).until(()->intake.isStalled()),
                intake.eject(()->2).withTimeout(0.1),
                intake.stop()

            ).repeatedly()).until(() -> Timer.getFPGATimestamp() - start_time > 0.4),
            superstruct.manual(() -> -1, () ->0).until(() -> superstruct.getWrist().atGoal(-0.1)),
            Commands.runOnce(() -> superstruct.disable_manual()),
            intake.stop(),
            new SuperToState(superstruct, 0.8, SuperPreset.ALGAE_STOW.getState())
        );
    }
}
