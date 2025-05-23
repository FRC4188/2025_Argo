package frc.robot.commands.scoring;

import java.lang.reflect.Field;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.AllianceFlip;
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
            presetGoal = true;
        }
        
        @Override
        public void factory() {
            if (!presetGoal)  {goal = AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);}

            FieldConstant.Reef.AlgaeSource.asources.remove(goal);
            int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);
            
            if (height == 3) {
                preset = SuperPreset.L3_ALGAE;
            } else {
                preset = SuperPreset.L2_ALGAE;
            }

            scoring = 
                Commands.sequence(
                    Commands.parallel(
                        Commands.sequence(
                            new DriveTo(drive, goal),
                            Commands.runOnce(drive::stopWithX, drive)
                        ),
                        Commands.sequence(
                            new WaitUntilCommand(() -> AllianceFlip.flipDS(drive.getPose()).getTranslation().getDistance(goal.getTranslation()) < 2),
                            new SuperToState(superstruct, 0, preset.getState())
                        )
                    ),
                    intake.ingest(() -> 7).withTimeout(1.5),
                    intake.stop()
                );
        }
    }


    public static class algaeProcess extends AutoScore {
        public algaeProcess(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = FieldConstant.Processor.processor_goal;
        }

        
        @Override
        public void factory() {
            preset = SuperPreset.PROCESSOR;
            
            scoring = 
                Commands.sequence(
                    Commands.parallel(
                        Commands.sequence(
                            new DriveTo(drive, goal),
                            Commands.runOnce(drive::stopWithX, drive)
                        ),
                        Commands.sequence(
                            new WaitCommand(0.25),
                            new SuperToState(superstruct, 0.5, SuperPreset.PROCESSOR.getState())
                        )
                    ),
                    intake.eject(() -> 10).withTimeout(1),
                    intake.stop()
                );

        }
    }
    //TODO: MIDMIDMIDMIDMIDMDID

    public static class algaeNet extends AutoScore {
        public algaeNet(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
        }

        public algaeNet(Pose2d pose, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
            presetGoal = true;
        }

        public algaeNet(Optional<Pose2d> pose, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            if(pose.isPresent()) goal = pose.get();
            presetGoal = true;
        }
        
        @Override
        public void factory() {
            if (!presetGoal)  {goal = AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Net.nscores);}
            FieldConstant.Net.nscores.remove(goal);

            scoring = 
                Commands.sequence(
                    new DriveTo(drive, goal),
                    Commands.runOnce(drive::stopWithX, drive), 
                    new ScoreNet(superstruct, intake)
                );
        }
    }

    public static Command pushLeave(Drive drive){
        return drive.sysIdDynamic(Direction.kForward).withTimeout(5);
    }

    public static class coralScore extends AutoScore {

        public coralScore(Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
        }

        public coralScore(Pose2d pose, Drive drive, Superstructure superstructure, Intake intake) {
            this.drive = drive;
            this.superstruct = superstructure;
            this.intake = intake;
            goal = pose;
            presetGoal = true;
        }

        @Override
        public void factory() {
            if (!presetGoal)  {goal = AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);}

            FieldConstant.Reef.AlgaeSource.asources.remove(goal);
            int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);
            
            if (height == 3) {
                preset = SuperPreset.L3_ALGAE;
            } else {
                preset = SuperPreset.L2_ALGAE;
            }

            scoring = 
                Commands.sequence(
                    Commands.race(
                        //intake.ingest(() -> 5),
                        Commands.parallel(
                            Commands.sequence(
                                new DriveTo(drive, goal),
                                Commands.runOnce(drive::stopWithX, drive)
                            ),
                            Commands.sequence(
                                new WaitUntilCommand(() -> AllianceFlip.flipDS(drive.getPose()).getTranslation().getDistance(goal.getTranslation()) < 2),
                                new SuperToState(superstruct, 0, SuperPreset.L1_CORAL.getState())
                            )
                        )
                    ),
                    new SuperToState(superstruct, 0, preset.getState()),
                    intake.ingest(() -> 7).withTimeout(0.7),
                    intake.stop()
                );
            
        }
    }
}