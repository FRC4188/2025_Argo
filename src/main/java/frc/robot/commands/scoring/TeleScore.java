package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.superstructure.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.FieldConstant;

public class TeleScore extends Command{
    protected Drive drive;
    protected Superstructure superstruct;
    protected Pose2d goal;
    protected SuperPreset preset;
    protected Command scoring;
    protected boolean presetGoal = false;

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

    public static class Score extends SequentialCommandGroup {
        
        public Score(Pose2d goal, SuperState state, Drive drive, Superstructure superstructure) {
            addCommands(
                new DriveToPose(drive, () -> new Pose2d(drive.getPose().getTranslation(), goal.getRotation())),
                new SuperToState(superstructure, state)
            );
        }
    }

    public static class coralScore extends TeleScore {

        public coralScore(int level, Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
    
            switch (level) {
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

        public coralScore(Pose2d pose, int bar, Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
            goal = pose;
            presetGoal = true;
    
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
            if (!presetGoal) goal = drive.getPose().nearest(FieldConstant.Reef.CoralGoal.cgoals);

            scoring = new Score(goal, preset.getState(), drive, superstruct);
        }
    }

    public static class algaeSource extends TeleScore {

        public algaeSource(Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
        }

        public algaeSource(Pose2d pose, Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
            goal = pose;
            presetGoal = true;
        }
        
        @Override
        public void factory() {
            if (!presetGoal) goal = drive.getPose().nearest(FieldConstant.Reef.AlgaeSource.asources);
            
            boolean flip = Math.abs(drive.getRotation().minus(goal.getRotation()).getRadians()) > Math.PI/2;
            int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);
            Pose2d correctedgoal = goal;
            
            if (!flip && height == 1) {
                preset = SuperPreset.L3_ALGAE;
            } else if(!flip && height == 0) {
                preset = SuperPreset.L2_ALGAE;
            } else if (flip && height == 1) {
                preset = SuperPreset.L3_ALGAE_REVERSE;
                correctedgoal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
            } else if (flip && height == 0) {
                preset = SuperPreset.L2_ALGAE_REVERSE;
                correctedgoal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
            } else {
                scoring = new Command() {};
            }

            scoring = new Score(correctedgoal, preset.getState(), drive, superstruct);
        }
    }

    public static class coralSource extends TeleScore {

        public coralSource(Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
        }

        public coralSource(Pose2d pose, Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
            goal = pose;
            presetGoal = true;
        }
        
        @Override
        public void factory() {
            if (!presetGoal) goal = drive.getPose().nearest(FieldConstant.Source.csources);
            
            Pose2d correctedGoal = goal;

            if (Math.abs(drive.getRotation().minus(goal.getRotation()).getRadians()) > Math.PI/2) {
                correctedGoal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
                preset = SuperPreset.SOURCE_REVERSE;
            } else {
                preset = SuperPreset.SOURCE;
            }

            scoring = new Score(correctedGoal, preset.getState(), drive, superstruct);
        }
    }

    public static class algaeScore extends TeleScore {

        public algaeScore(Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
            goal = FieldConstant.Processor.processor_goal;
        }

        
        @Override
        public void factory() {
            Pose2d correctedGoal = goal;

            if (superstruct.getState().getCartesian(false).getX() > 0) {
                correctedGoal = goal.transformBy(new Transform2d(Translation2d.kZero, Rotation2d.k180deg));
                preset = SuperPreset.PROCESSOR_REVERSE;
            } else {
                preset = SuperPreset.PROCESSOR;
            }

            scoring = new Score(correctedGoal, preset.getState(), drive, superstruct);
        }
    }
}
