package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.FieldConstant;

public final class AutoTests {
    private final static PathConstraints  constraints =  new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(3.6),
            DegreesPerSecond.of(TunerConstants.kSpeedAt12Volts.magnitude()),
            DegreesPerSecondPerSecond.of(862),
            Volts.of(12),
            false);

    // //runs fine, bit off
    // public static Command twoCoral(){
    //     PathPlannerPath a;
    //     try{
    //         a = PathPlannerPath.fromPathFile("2 Corals");
    //         return AutoBuilder.followPath(a);
    //     }catch(Exception e){
    //         e.printStackTrace();
    //     }
    //     return new SequentialCommandGroup();
    // }

    public static Command test1(Drive drive) {
        drive.setPose(new Pose2d(7.459, 4.160, Rotation2d.k180deg));
        return new DriveToPose(drive, () -> FieldConstant.Reef.AlgaeSource.mid_brg_src);
    }

    public static Command test2(Drive drive) {
        drive.setPose(new Pose2d(7.180, 7.550, Rotation2d.k180deg));
        return new DriveTo(drive, FieldConstant.Reef.AlgaeSource.right_src_src);
    }

    public static Command AG2Coral(Drive drive){
        drive.setPose(new Pose2d(7.459, 4.160, Rotation2d.k180deg));
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
}
