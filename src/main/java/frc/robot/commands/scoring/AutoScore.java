package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.FieldConstant;

public class AutoScore extends Command{
    protected Drive drive;
    protected Superstructure superstruct;
    protected Intake intake;
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
        public float happy_zone = 2;


        public Score(Pose2d goal, SuperState state,Command intakeCommand, Drive drive, Superstructure superstructure, double safe) {
            addCommands(
                new DriveTo(drive, goal).alongWith(
                    new SuperToState(superstructure, 0.5, SuperState.SuperPreset.ALGAE_STOW.getState())
                    .until(() -> drive.getPose().getTranslation().getDistance(goal.getTranslation()) <= happy_zone)
                    .andThen(new WaitUntilCommand(() -> drive.getPose().getTranslation().getDistance(goal.getTranslation()) <= happy_zone))
                    .andThen(new SuperToState(superstructure,safe, state))),
                intakeCommand
            );
        }
    }

    public static class algaeSource extends AutoScore {
        int level; 

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
            presetGoal = true;
        }

        public algaeSource(int level, Pose2d pose, Drive drive, Superstructure superstructure, Intake intake){
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
            presetGoal = true;
            this.level = level;
        }
        
        @Override
        public void factory() {
            if (!presetGoal) goal = drive.getPose().nearest(FieldConstant.Reef.AlgaeSource.asources);
            
            int height = level == 0 ? FieldConstant.Reef.AlgaeSource.algaeHeight(goal): this.level;
            Pose2d correctedgoal = goal;
            
            if (height == 3) {
                preset = SuperPreset.L3_ALGAE;
            } else if(height == 2) {
                preset = SuperPreset.L2_ALGAE;
            } else {
                scoring = new Command() {};
            }

            scoring = new Score(correctedgoal,  preset.getState(), intake.ingest().withTimeout(1.5), drive, superstruct, 0);
        }
    }


    public static class algaeScore extends AutoScore {
        Intake intake;

        public algaeScore(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = FieldConstant.Processor.processor_goal;
        }

        
        @Override
        public void factory() {
            Pose2d correctedGoal = goal;

            preset = SuperPreset.PROCESSOR;
            
            scoring = new Score(correctedGoal, preset.getState(), intake.ingest().withTimeout(1.5), drive, superstruct, 0.5);
        }
    }
}
