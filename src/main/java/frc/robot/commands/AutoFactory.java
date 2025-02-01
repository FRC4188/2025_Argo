package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.dyn4j.world.CollisionWorld;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.FollowPath;
import frc.robot.pathgen.PG_math;
import frc.robot.pathgen.PathGen;
import frc.robot.pathgen.fieldobjects.CircleFO;
import frc.robot.pathgen.fieldobjects.FOHandler;
import frc.robot.pathgen.fieldobjects.PolygonFO;
import frc.robot.pathgen.fieldobjects.RectFO;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Source;

public final class AutoFactory {
    private final static PathConstraints  constraints =  new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(3.6),
            DegreesPerSecond.of(TunerConstants.kSpeedAt12Volts.magnitude()),
            DegreesPerSecondPerSecond.of(862),
            Volts.of(12),
            false);
    
    public static TrajectoryConfig config = new TrajectoryConfig(
        TunerConstants.kSpeedAt12Volts.magnitude() / 2, 
        Constants.robot.MAX_ACCELERATION.magnitude() / 2);
        
    //run barely, slow and not accurate
    public static Command toBasetoSource(){
        Pose2d currPose = new Pose2d(FieldConstant.Cage.cage_positions[1].getX(), FieldConstant.Cage.cage_positions[1].getY(), new Rotation2d(Degrees.of(0)));
    
        Pose2d targetPose2d = new Pose2d(FieldConstant.Reef.Base.left_brg_corner, new Rotation2d(Degrees.of(0)));
        Pose2d targetPose2d1 = new Pose2d(FieldConstant.Reef.Base.right_brg_corner, new Rotation2d(Degrees.of(0)));
        Pose2d source = FieldConstant.Source.right_srcs[4];
    
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

    //not moving like irl for sim
    public static Command follow2Coral(Drive drive){
        return new FollowPath(
            TrajectoryGenerator.generateTrajectory(
                drive.getPose(), 
                List.of(
                    Reef.Base.left_brg_corner,
                    Source.left_srcs[4].getTranslation(),
                    Reef.Base.right_field_corner),
                new Pose2d(Reef.Base.right_field_corner, drive.getRotation()),
                config),
            
            new Rotation2d(Degrees.of(0)),
            drive
            );
    }

    public static void pathgeninit() {
        new PolygonFO(
            FieldConstant.Reef.Base.left_brg_corner,
            FieldConstant.Reef.Base.right_brg_corner,
            FieldConstant.Reef.Base.right_field_corner,
            FieldConstant.Reef.Base.right_src_corner,
            FieldConstant.Reef.Base.left_src_corner,
            FieldConstant.Reef.Base.left_field_corner);
        
        new PolygonFO(
            FieldConstant.Field.all_wall_left_corner,
            FieldConstant.Field.alliance_left_corner,
            FieldConstant.Field.alliance_right_corner,
            FieldConstant.Field.all_wall_right_corner,
            FieldConstant.Field.opp_wall_right_corner,
            FieldConstant.Field.opposing_right_corner,
            FieldConstant.Field.opposing_left_corner,
            FieldConstant.Field.opp_wall_left_corner
        );

        new RectFO(
            (float) FieldConstant.field_center_x,
            (float) FieldConstant.field_center_y,
            (float) FieldConstant.Field.brg_length,
            (float) FieldConstant.Field.brg_width
        );

        new CircleFO(
            (float) FieldConstant.Elem_Locations.corals_locations[0].getX(),
            (float) FieldConstant.Elem_Locations.corals_locations[0].getY(),
            (float) FieldConstant.algae_radius
        );

        PathGen.getInstance().update_grid_fo();
    }

    public static void printObstacles() {
        Translation2d[] borders = {
            FieldConstant.Field.all_wall_left_corner,
            FieldConstant.Field.alliance_left_corner,
            FieldConstant.Field.alliance_right_corner,
            FieldConstant.Field.all_wall_right_corner,
            FieldConstant.Field.opp_wall_right_corner,
            FieldConstant.Field.opposing_right_corner,
            FieldConstant.Field.opposing_left_corner,
            FieldConstant.Field.opp_wall_left_corner
        };

        Translation2d[] reef = {
            FieldConstant.Reef.Base.left_brg_corner,
            FieldConstant.Reef.Base.right_brg_corner,
            FieldConstant.Reef.Base.right_field_corner,
            FieldConstant.Reef.Base.right_src_corner,
            FieldConstant.Reef.Base.left_src_corner,
            FieldConstant.Reef.Base.left_field_corner
        };

        PG_math.printpose(reef);
        PG_math.printpose(borders);
        PG_math.printpose(FieldConstant.Source.left_srcs);
        PG_math.printpose(FieldConstant.Source.right_srcs);
    }


    public static Command AG2Coral(Drive drive){
        pathgeninit();
        
        //printObstacles();

        return Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_brg_src, config),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.left_brg_left, config),
            new DriveTo(drive, FieldConstant.Source.left_srcs[6], config),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.mid_brg_right, config),
            new DriveTo(drive, FieldConstant.Source.right_srcs[7], config),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.left_brg_left, config)
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
