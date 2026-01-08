package frc.robot.commands.tracking;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.tracking.Tracking;
import java.util.function.Supplier;

public class TrackCommands {
  public static ProfiledPIDController angleController =
      new ProfiledPIDController(
          Constants.robot.ANGLE_PID.kP,
          Constants.robot.ANGLE_PID.kI,
          Constants.robot.ANGLE_PID.kD,
          new TrapezoidProfile.Constraints(
              Constants.robot.ANGLE_MAX_VELOCITY, Constants.robot.ANGLE_MAX_ACCELERATION));

  public static void updateAnglePID(double kp, double ki, double kd, double kg) {
    angleController =
        new ProfiledPIDController(
            kp,
            ki,
            kd,
            new TrapezoidProfile.Constraints(
                Constants.robot.ANGLE_MAX_VELOCITY, Constants.robot.ANGLE_MAX_ACCELERATION));

    // angleController.setTolerance(0.02);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private TrackCommands() {}

  public static Command rotateTowardsTarget(
    Drive drive, Tracking track, Supplier<Rotation2d> rotationSupplier) {

    return Commands.none();
  }
}
