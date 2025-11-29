package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CSPLib.pathgen.PathGen;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import org.littletonrobotics.junction.Logger;

public class DriveToPath extends Command {

  private double start_time = 0;
  private Trajectory traj;
  private TrajectoryConfig config;
  private Drive drive;
  private Pose2d end_goal;
  private HolonomicDriveController controller;

  // flipped drive, unflipped goal
  public DriveToPath(Drive drive, Pose2d goal) {
    addRequirements(drive);

    this.drive = drive;
    config =
        new TrajectoryConfig(
            TunerConstants.kSpeedAt12Volts.magnitude() * 0.5,
            Constants.robot.MAX_ACCELERATION.magnitude() * 0.7);
    end_goal = goal;

    ProfiledPIDController angleController =
        new ProfiledPIDController(
            Constants.robot.ANGLE_PID.kP,
            Constants.robot.ANGLE_PID.kI,
            Constants.robot.ANGLE_PID.kD,
            new TrapezoidProfile.Constraints(
                Constants.robot.ANGLE_MAX_VELOCITY, Constants.robot.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    controller =
        new HolonomicDriveController(
            new PIDController(5.0, 0, 0), new PIDController(5.0, 0, 0), angleController);
  }

  @Override
  public void initialize() {
    traj =
        PathGen.getInstance()
            .generateTrajectory(AllianceFlip.flipDS(drive.getPose()), end_goal, config);

    start_time = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    State curstate = traj.sample(Timer.getFPGATimestamp() - start_time);

    curstate.poseMeters = AllianceFlip.flipDS(curstate.poseMeters);

    drive.runVelocity(
        controller.calculate(drive.getPose(), curstate, curstate.poseMeters.getRotation()));

    Logger.recordOutput("Drive/Traj_Pose", curstate.poseMeters);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - start_time >= traj.getTotalTimeSeconds() + 0.5;
  }
}
