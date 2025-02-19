package frc.robot.commands.scoring;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.Intake.Mode;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.util.FieldConstant;

public class AutoScore {

    public static List<Pose2d> cgoals = Arrays.asList(FieldConstant.Reef.CoralGoal.coralGoals);
    public static List<Pose2d> agoals = Arrays.asList(FieldConstant.Reef.AlgaeSource.algGoal);

    public static Command coralScore(int bar, Drive drive, Superstructure superstruct, Intake intake) {
        cgoals.sort((a, b) -> Double.compare(
            drive.getPose().getTranslation().getDistance(a.getTranslation()), 
            drive.getPose().getTranslation().getDistance(b.getTranslation())));
        
        Pose2d closest = cgoals.get(0);

        SuperPreset preset;
        switch (bar) {
            case 2:
                preset = SuperPreset.L2_CORAL;
                break;
            case 3:
                preset = SuperPreset.L3_CORAL;
                break;
            case 4:
                preset = SuperPreset.L4_CORAL;
                break;
        
            default:
                preset = SuperPreset.L2_CORAL;
                break;
        }

        return new Score(closest, preset.getState(), intake.eject(), drive, superstruct, intake);
    }

    public static Command algaeScore(Drive drive, Superstructure superstruct, Intake intake) {
        agoals.sort((a, b) -> Double.compare(
            drive.getPose().getTranslation().getDistance(a.getTranslation()), 
            drive.getPose().getTranslation().getDistance(b.getTranslation())));
        
        Pose2d closest = agoals.get(0);
        
        SuperPreset preset;

        if (FieldConstant.Reef.AlgaeSource.algaeHeight(closest) == 1 && superstruct.getState().getCartesian(false).getX() >= 0) {
            preset = SuperPreset.L3_ALGAE;
            
        } else if (FieldConstant.Reef.AlgaeSource.algaeHeight(closest) == 1 && superstruct.getState().getCartesian(false).getX() <= 0){
            preset = SuperPreset.L3_ALGAE_REVERSE;
            closest.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
        } else if (FieldConstant.Reef.AlgaeSource.algaeHeight(closest) == 0 && superstruct.getState().getCartesian(false).getX() >= 0){
            preset = SuperPreset.L2_ALGAE;
            
        } else if(FieldConstant.Reef.AlgaeSource.algaeHeight(closest) == 0 && superstruct.getState().getCartesian(false).getX() <= 0) {
            preset = SuperPreset.L2_ALGAE_REVERSE;
            closest.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
        } else {
            return new Command() {};
        }
        
        return new Score(closest, preset.getState(), intake.ingest(Mode.ALGAE), drive, superstruct, intake);
    }

}
