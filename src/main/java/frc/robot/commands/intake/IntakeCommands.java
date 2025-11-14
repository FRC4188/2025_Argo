package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {

  public static Command driveIntake(Intake intake, DoubleSupplier input) {

    return Commands.runOnce(
        () -> {
          intake.setVelocityRPM(input.getAsDouble() * Constants.IntakeConstants.kMaxVel);
        },
        intake);
  }

  private static double start_time = 0.0;

  public static Command driveVolts(Intake intake, DoubleSupplier input) {

    return Commands.runOnce(
        () -> {
          intake.runVolts(input.getAsDouble() * 12);
        },
        intake);
  }

  public static Command unstickIntake(Intake intake) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
            Commands.run(() -> intake.runVolts(12), intake).until(() -> intake.isStalled()),
            Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
            Commands.run(() -> intake.runVolts(-2), intake).withTimeout(0.2))
        .until(() -> Timer.getFPGATimestamp() - start_time > 0.4);
  }
}
