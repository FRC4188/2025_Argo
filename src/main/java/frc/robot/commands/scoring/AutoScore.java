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

public class AutoScore extends Command {
    protected Drive drive;
    protected Superstructure superstruct;
    protected Intake intake;
    protected Pose2d goal;
    protected SuperPreset preset;
    protected Command scoring;

    public void factory() {

    }

    public final void initialize() {
        factory();
        scoring.initialize();
    }

    public final void execute() {
        scoring.execute();
    }

    public final boolean isFinished() {
        return scoring.isFinished();
    }

    public final void end(boolean interrupted) {
        scoring.end(interrupted);
    }

    public static class coralScore extends AutoScore {

        public coralScore(int bar, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
    
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
        }

        public coralScore(Pose2d pose, int bar, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
    
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
        }
        
        @Override
        public void factory() {
            if (goal == null) goal = drive.getPose().nearest(FieldConstant.Reef.CoralGoal.cgoals);
    
            scoring = new Score(goal, preset.getState(), intake.eject(), drive, superstruct, intake);
        }
    }

    public static class algaeSource extends AutoScore {

        public algaeSource(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
        }

        public algaeSource(Pose2d pose, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
        }
        
        @Override
        public void factory() {
            

            if (goal == null) goal = drive.getPose().nearest(FieldConstant.Reef.AlgaeSource.asources);
            
            boolean flip = Math.abs(drive.getRotation().minus(goal.getRotation()).getRadians()) > Math.PI/2;
            int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);
    
            if (!flip && height == 1) {
                preset = SuperPreset.L3_ALGAE;
            } else if(!flip && height == 0) {
                preset = SuperPreset.L2_ALGAE;
            } else if (flip && height == 1) {
                preset = SuperPreset.L3_ALGAE_REVERSE;
                goal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
            } else if (flip && height == 0) {
                preset = SuperPreset.L2_ALGAE_REVERSE;
                goal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
            } else {
                scoring = new Command() {};
            }

            scoring = new Score(goal, preset.getState(), intake.ingest(Mode.ALGAE).andThen(() -> FieldConstant.Reef.AlgaeSource.asources.remove(goal)), drive, superstruct, intake);
        }
    }

    public static class coralSource extends AutoScore {

        public coralSource(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
        }

        public coralSource(Pose2d pose, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
        }
        
        @Override
        public void factory() {
            if (goal == null) goal = drive.getPose().nearest(FieldConstant.Source.csources);
    
            if (Math.abs(drive.getRotation().minus(goal.getRotation()).getRadians()) > Math.PI/2) {
                goal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
                preset = SuperPreset.SOURCE_REVERSE;
            } else {
                preset = SuperPreset.SOURCE;
            }

            scoring = new Score(goal, preset.getState(), intake.ingest(Mode.CORAL), drive, superstruct, intake);
        }
    }

    public static class algaeScore extends AutoScore {

        public algaeScore(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = FieldConstant.Processor.processor_goal;
        }

        
        @Override
        public void factory() {
            if (superstruct.getState().getCartesian(false).getX() > 0) {
                goal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
                preset = SuperPreset.PROCESSOR_REVERSE;
            } else {
                preset = SuperPreset.PROCESSOR;
            }

            scoring = new Score(goal, preset.getState(), intake.eject(), drive, superstruct, intake);
        }
    }

}
