package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.FollowPath;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Source;

public final class AutoTests {
    private final static PathConstraints  constraints =  new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(3.6),
            DegreesPerSecond.of(TunerConstants.kSpeedAt12Volts.magnitude()),
            DegreesPerSecondPerSecond.of(862),
            Volts.of(12),
            false);
        
    //run barely, slow and not accurate
    public static Command toBasetoSource(){
        Pose2d currPose = new Pose2d(FieldConstant.Cage.cage_positions[1].getX(), FieldConstant.Cage.cage_positions[1].getY(), new Rotation2d(Degrees.of(0)));
    
        Pose2d targetPose2d = new Pose2d(FieldConstant.Reef.Base.left_brg_corner, new Rotation2d(Degrees.of(0)));
        Pose2d targetPose2d1 = new Pose2d(FieldConstant.Reef.Base.right_brg_corner, new Rotation2d(Degrees.of(0)));
        Pose2d source = FieldConstant.Source.right_srcs[0];
    
        List<Waypoint> pts  = PathPlannerPath.waypointsFromPoses(currPose,targetPose2d, source, targetPose2d1);
            
            

        return AutoBuilder.pathfindThenFollowPath(
            new PathPlannerPath(
                pts,
                constraints, 
                new IdealStartingState(0, currPose.getRotation()),
                new GoalEndState(0.0, targetPose2d1.getRotation())),

            constraints
        );
    }

    //runs fine, bit off
    public static Command twoCoral(){
        PathPlannerPath a;
        try{
            a = PathPlannerPath.fromPathFile("2 Corals");
            return AutoBuilder.followPath(a);
        }catch(Exception e){
            e.printStackTrace();
        }
        return new SequentialCommandGroup();
    }

    public static Command AG2Coral(Drive drive){
        
        System.out.println(FieldConstant.Processor.processor_wall);
        System.out.println(FieldConstant.Processor.processor_goal);

        return Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_brg_src),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.left_brg_left),
            new DriveTo(drive, FieldConstant.Source.left_srcs[0]),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.mid_brg_right),
            new DriveTo(drive, FieldConstant.Source.right_srcs[0]),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.left_brg_left),
            new DriveTo(drive, FieldConstant.Processor.processor_goal)
        );
        
    }

    //literally FLAWLESS
    public static Command drive2Corals(Drive drive){
        return Commands.sequence(
            new DriveToPose(drive, () -> new Pose2d(5.245, 5.276, new Rotation2d(Degrees.of(-120)))),
            new WaitCommand(0.5),
            // new DriveToPose(drive, () -> new Pose2d(1.383,7.039, new Rotation2d(Degrees.of(-55)))),
            // new WaitCommand(0.5),
            new DriveToPose(drive, () -> new Pose2d(3.765,5.240, new Rotation2d(Degrees.of(-60))))
        );
    }

    //add cmds for pathplanner events
    public static final HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(            
            Map.entry("Delay", new WaitCommand(2.0))
      ));
}
