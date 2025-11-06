package frc.robot.commands.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperState.SuperPreset;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;

//TODO temp until intake is fully in place
public class ScoreNet extends SequentialCommandGroup {
  private double start_time = 0.0;

  public ScoreNet(SuperStructure superstruct, Intake intake) {
      
      addCommands(
          SuperCommands.superToState(superstruct, SuperPreset.NET.getState(), Rotation2d.fromRadians(0.75)),
          (Commands.sequence(
              Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
              Commands.run(() -> intake.runVolts(12), intake).until(()->intake.isStalled()),
              Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
              Commands.run(() -> intake.runVolts(-2), intake).withTimeout(0.2)
          ).repeatedly()).until(() -> Timer.getFPGATimestamp() - start_time > 0.4),
          SuperCommands.superDrive(superstruct, () -> 0.0, () -> -1).until(() -> superstruct.atWristGoal(Rotation2d.fromRadians(-0.1))),
          Commands.runOnce(() -> intake.stop()),
          SuperCommands.superToState(superstruct, SuperPreset.ALGAE_STOW.getState())
      );
  }
}