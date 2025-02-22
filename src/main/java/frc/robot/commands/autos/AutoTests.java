package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.util.FieldConstant;

public final class AutoTests {
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
        return Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_brg_src),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.left_brg_left),
            new DriveTo(drive, FieldConstant.Source.left_src_mid),
            new DriveTo(drive, FieldConstant.Reef.CoralGoal.mid_brg_right),
            new DriveTo(drive, FieldConstant.Source.right_src_mid),
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
