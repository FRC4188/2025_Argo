package frc.robot.commands.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.superstructure.SuperStructure;

// TODO temp until intake is fully in place
public class ScoreNet extends SequentialCommandGroup {
  private double start_time = 0.0;

  public ScoreNet(SuperStructure superstruct, Intake intake) {

    addCommands(
        SuperCommands.superToState(
            superstruct, SuperPreset.NET.getState(), Rotation2d.fromRadians(0.75)),
        IntakeCommands.unstickIntake(intake),
        SuperCommands.superDrive(superstruct, () -> 0.0, () -> -0.5)
            .until(() -> superstruct.atWristGoal(Rotation2d.fromRadians(-0.1))),
        SuperCommands.superToState(superstruct, SuperPreset.ALGAE_STOW.getState()),
        Commands.runOnce(intake::stop, intake));
  }
}
