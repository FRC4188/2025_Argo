package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
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
        
        public Score(Pose2d goal, SuperState state, Drive drive, Superstructure superstructure, double safe) {
            addCommands(
                new DriveToPose(drive, () -> new Pose2d(drive.getPose().getTranslation(), goal.getRotation())),
                new SuperToState(superstructure, safe, state)
            );
        }

        public Score(SuperState state, Superstructure superstructure, Command intake, double safe){
            addCommands(
                new SuperToState(superstructure, safe, state),
                intake
            );
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
            
            int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);
            Pose2d correctedgoal = goal;
            
            if (height == 1) {
                preset = SuperPreset.L3_ALGAE;
            } else if(height == 0) {
                preset = SuperPreset.L2_ALGAE;
            } else {
                scoring = new Command() {};
            }

            scoring = new Score(correctedgoal, preset.getState(), drive, superstruct, 0);
        }
    }

    public static class algaeReef extends TeleScore {
        Intake intake;
        int level;

        public algaeReef(int level, Superstructure superstructure, Intake intake) {
            this.superstruct = superstructure;
            this.intake = intake;
            this.level = level;
        }
        
        @Override
        public void factory() {
            preset = level == 2? SuperPreset.L2_ALGAE : SuperPreset.L3_ALGAE;
            scoring = new Score(preset.getState(), superstruct, intake.ingest(()-> 1), 0);
        }
    }


    public static class algaeScore extends TeleScore {
        Intake intake;

        public algaeScore(Drive drive, Superstructure superstructure) {
            this.drive = drive;
            this.superstruct = superstructure;
            goal = FieldConstant.Processor.processor_goal;
        }

        public algaeScore(Superstructure superstructure, Intake intake, double safe){
            this.superstruct = superstructure;
            this.intake = intake;
            preset = SuperPreset.PROCESSOR;
        }

        
        @Override
        public void factory() {
            Pose2d correctedGoal = goal;

            preset = SuperPreset.PROCESSOR;
            

            if(correctedGoal != null) scoring = new Score(correctedGoal, preset.getState(), drive, superstruct, 0.5);
            else scoring = new Score(preset.getState(), superstruct,intake.eject(() -> 1), 1);
        }
    }
}
