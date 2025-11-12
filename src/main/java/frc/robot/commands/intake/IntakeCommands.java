package frc.robot.commands.intake;

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

  public static Command driveVolts(Intake intake, DoubleSupplier input) {

    return Commands.runOnce(
        () -> {
          intake.runVolts(input.getAsDouble() * 12);
        },
        intake);
  }
}
