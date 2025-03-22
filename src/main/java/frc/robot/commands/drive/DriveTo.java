package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.autos.pathgen.PG_math;
import frc.robot.commands.autos.pathgen.PathGen;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.AllianceFlip;

public class DriveTo extends Command {

    double start_time = 0;
    Supplier<Pose2d> goalPose;
    Trajectory traj;
    TrajectoryConfig config;
    DriveToPose driving;
    Drive drive;

    //flipped drive, unflipped goal
    public DriveTo(Drive drive, Pose2d goal) {
        this.drive = drive;
        config = new TrajectoryConfig(
            TunerConstants.kSpeedAt12Volts.magnitude() * 0.5, 
            Constants.robot.MAX_ACCELERATION.magnitude() * 0.5);
        goalPose = () -> goal;
    }

    @Override
    public void initialize() {
        traj = PathGen.getInstance().generateTrajectory(AllianceFlip.flipDS(drive.getPose()),goalPose.get(), config);

        if (traj.getStates().isEmpty()) {
            goalPose = () -> drive.getPose();
        } else {
            goalPose = () -> AllianceFlip.flipDS(traj.sample(Timer.getFPGATimestamp() - start_time).poseMeters);

            driving = new DriveToPose(drive, goalPose);
        }
        System.out.println("trajector:");
        // System.out.println(traj);
        
        driving.repeatedly().schedule();
        //driving.initialize();
        System.out.println("Driving ....");
        start_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        // System.out.print("Set goal ");
        // PG_math.printpose(goalPose.get());
        // System.out.print("current goal ");
        // PG_math.printpose(drive.getPose());

        //driving.execute();

        // if (driving.isFinished()) {
        //     driving.initialize();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (driving != null) driving.cancel();
        //driving.end(interrupted);
        drive.stopWithX();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - start_time > traj.getTotalTimeSeconds()  +1;
    }


}
