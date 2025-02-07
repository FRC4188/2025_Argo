package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.pathgen.PathGen;
import frc.robot.subsystems.drivetrain.Drive;

public class DriveTo extends Command {

    Timer timer;
    Supplier<Pose2d> goalPose;
    Trajectory traj;
    TrajectoryConfig config;
    DriveToPose driving;
    Drive drive;

    public DriveTo(Drive drive, Pose2d goal, TrajectoryConfig config) {
        this.drive = drive;
        this.config = config;

        timer = new Timer();

        goalPose = () -> goal;
    }

    @Override
    public void initialize() {
        traj = PathGen.getInstance().generateTrajectory(drive.getPose(), goalPose.get(), config);

        if (traj.getStates().isEmpty()) {
            goalPose = () -> drive.getPose();
        } else {
            goalPose = () -> traj.sample(timer.get()).poseMeters;

            driving = new DriveToPose(drive, goalPose);
        }

        

        timer.start();

        driving.repeatedly().schedule();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        if (driving != null) driving.cancel();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(traj.getTotalTimeSeconds() + 1);
    }


}
