package frc.robot.commands.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.*;
import java.util.function.DoubleSupplier;

public class SuperCommands {

  public static Command superDrive(
      SuperStructure superstruct, DoubleSupplier eleControl, DoubleSupplier wristControl) {

    return Commands.runEnd(
        () -> {
          if (eleControl.getAsDouble() != 0.0) {
            superstruct.runElevator(12 * eleControl.getAsDouble());
          } else {
            superstruct.setElevator(superstruct.getState().getEleHeight());
          }

          if (wristControl.getAsDouble() != 0.0) {
            superstruct.runWrist(12 * wristControl.getAsDouble());
          } else {
            superstruct.setWrist(superstruct.getState().getWristAngle());
          }
        },
        () -> {
          superstruct.setState(superstruct.getState());
        },
        superstruct);
  }

  public static Command superToState(
      SuperStructure superstruct, SuperState superstate, Rotation2d safeAngle) {

    return Commands.sequence(
        Commands.runOnce(() -> superstruct.setWrist(safeAngle), superstruct),
        Commands.waitUntil(() -> superstruct.atWristGoal(safeAngle)),
        Commands.runOnce(() -> superstruct.setElevator(superstate.getEleHeight()), superstruct),
        Commands.waitUntil(() -> superstruct.atElevatorGoal(superstate.getEleHeight(), 0.2)),
        Commands.runOnce(() -> superstruct.setWrist(superstate.getWristAngle()), superstruct),
        Commands.waitUntil(() -> superstruct.atWristGoal(superstate.getWristAngle())));
  }
}
