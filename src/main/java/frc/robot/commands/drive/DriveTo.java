package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CSPLib.pathgen.PathGen;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.function.Supplier;

public class DriveTo extends Command {

  private double start_time = 0;
  private Supplier<Pose2d> goalPose;
  private Trajectory traj;
  private TrajectoryConfig config;
  private DriveToPose driving;
  private Drive drive;
  private Pose2d end_goal;

  // flipped drive, unflipped goal
  public DriveTo(Drive drive, Pose2d goal) {
    addRequirements(drive);

    this.drive = drive;
    config =
        new TrajectoryConfig(
            TunerConstants.kSpeedAt12Volts.magnitude() * 0.8,
            Constants.robot.MAX_ACCELERATION.magnitude() * 0.4);
    end_goal = goal;
  }

  @Override
  public void initialize() {
    traj =
        PathGen.getInstance()
            .generateTrajectory(AllianceFlip.flipDS(drive.getPose()), end_goal, config);

    if (traj.getStates().isEmpty()) {
      goalPose = () -> drive.getPose();
    } else {
      goalPose =
          () -> AllianceFlip.flipDS(traj.sample(Timer.getFPGATimestamp() - start_time).poseMeters);

      driving = new DriveToPose(drive, goalPose);
    }

    driving.initialize();

    start_time = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    driving.execute();
  }

  @Override
  public void end(boolean interrupted) {
    driving.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - start_time >= traj.getTotalTimeSeconds() + 1
        || AllianceFlip.flipDS(drive.getPose())
                .getTranslation()
                .getDistance(end_goal.getTranslation())
            <= Units.inchesToMeters(1.0);
  }
}
