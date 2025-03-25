package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;

public class ScoreNet extends SequentialCommandGroup {

    public ScoreNet(Superstructure superstruct, Intake intake) {
        addRequirements(superstruct, intake);

        addCommands(
            new SuperToState(superstruct, 1, SuperPreset.NET.getState()),
            Commands.sequence(
                intake.ingest(() -> 10).until(()->intake.isStalled()),
                intake.eject(()->5).withTimeout(0.2),
                intake.stop()
            ).repeatedly().until(()->!intake.isIn()),
            new SuperToState(superstruct, 0),
            intake.stop(),
            new SuperToState(superstruct, 0.5, SuperPreset.ALGAE_STOW.getState())
        );
    } 
}
