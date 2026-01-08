package frc.robot.commands.tracking;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.tracking.Tracking;

public class RotateToTarget extends Command {
    private final Drive drive;
    private final Tracking track;
    private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          Constants.robot.ANGLE_PID.kP,
          Constants.robot.ANGLE_PID.kI,
          Constants.robot.ANGLE_PID.kD,
          new TrapezoidProfile.Constraints(
              Constants.robot.ANGLE_MAX_VELOCITY, Constants.robot.ANGLE_MAX_ACCELERATION));

    public RotateToTarget(Drive drive, Tracking track, Supplier<Rotation2d> rotationSupplier) {
        this.drive = drive;
        this.track = track;

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!track.hasTarget()) {
            drive.stop();
            return;
        }

        double omega = angleController.calculate(
            track.getTarget()
            .getRotation()
            .getRadians(), 
            0.0);

        omega = MathUtil.clamp(omega, -0.6, 0.6);

        drive.runVelocity(
            new ChassisSpeeds(
                0.0,
                0.0,
                omega
            )
        );
    }

    @Override
    public boolean isFinished() {
        return track.hasTarget() && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
