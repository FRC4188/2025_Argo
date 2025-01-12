// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;

public class FollowPath extends Command {

  private HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(Constants.robot.DRIVE_PID.kP,Constants.robot.DRIVE_PID.kI,Constants.robot.DRIVE_PID.kD), 
    new PIDController(Constants.robot.DRIVE_PID.kP,Constants.robot.DRIVE_PID.kI,Constants.robot.DRIVE_PID.kD), 
    new ProfiledPIDController(Constants.robot.TURN_PID.kP ,Constants.robot.TURN_PID.kI, Constants.robot.TURN_PID.kD,
    new Constraints(TunerConstants.kSpeedAt12Volts.magnitude(), 12.6)));

    private final Trajectory trajectory;
    private final Rotation2d heading;
    private final Timer timer = new Timer();

    private final Drive drive;
  /** Creates a new Follower. */
  public FollowPath(Trajectory trajectory, Rotation2d heading, Drive drive) {

    addRequirements(drive);

    this.trajectory = trajectory;
    this. heading = heading;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(controller.calculate(
      drive.getPose(), trajectory.sample(timer.get()), heading));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}