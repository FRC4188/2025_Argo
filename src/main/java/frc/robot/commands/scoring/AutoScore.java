package frc.robot.commands.scoring;

import static frc.robot.util.FieldConstant.Reef.AlgaeSource.asources;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveTo;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;

public class AutoScore {

    public static class algaeSource extends SequentialCommandGroup {

        private double happy_zone = 1;
        private Pose2d end_goal;
        private SuperPreset preset;
        private Supplier<Command> factory = ()-> new Command() {};

        public algaeSource(Drive drive, Superstructure superstruct, Intake intake) {
            addRequirements(drive, superstruct, intake);
            end_goal = drive.getPose();

            addCommands(
                Commands.runOnce(
                    () -> {
                        end_goal = AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);
                        asources.remove(end_goal);

                        int height = FieldConstant.Reef.AlgaeSource.algaeHeight(end_goal);
            
                        if (height == 3) {
                            preset = SuperPreset.L3_ALGAE;
                        } else {
                            preset = SuperPreset.L2_ALGAE;
                        }

                        factory = ()-> (new DriveTo(drive, end_goal).alongWith(
                            new WaitUntilCommand(
                                () -> AllianceFlip.flipDS(
                                    drive.getPose()).getTranslation().getDistance(end_goal.getTranslation()) < happy_zone)
                            .andThen(new SuperToState(superstruct,0, preset.getState())))
                            .andThen(intake.ingest(()->4).withTimeout(1))
                            .andThen(intake.stop()));

                        factory.get().initialize();
                    }  
                ),

                Commands.run(() -> (factory.get()).execute()).until(() -> factory.get().isFinished()),
                Commands.runOnce(() -> factory.get().end(false))
            );
        }

    }

    public static class algaeProcess extends SequentialCommandGroup {
        private Pose2d end_goal = FieldConstant.Processor.processor_goal;

        public algaeProcess(Drive drive, Superstructure superstruct, Intake intake) {
            addRequirements(drive, superstruct, intake);

            addCommands( 
                new DriveTo(drive, end_goal).alongWith(
                    new WaitUntilCommand(0.2).andThen(
                        new SuperToState(superstruct, 0.5, SuperPreset.ALGAE_STOW.getState())
                    )
                ),
                intake.eject(()->6).withTimeout(1.5),
                intake.stop()
            );

        }

    }

    public class algaeNet extends SequentialCommandGroup {
        private Pose2d end_goal;

        public algaeNet(Drive drive, Superstructure superstruct, Intake intake) {
            addRequirements(drive, superstruct, intake);

            addCommands( 
                Commands.runOnce(
                    () -> {
                        end_goal = drive.getPose().nearest(FieldConstant.Net.nscores);
                        FieldConstant.Net.nscores.remove(end_goal);
                    }
                ),
                new DriveTo(drive, end_goal).alongWith(
                    new WaitUntilCommand(0.2).andThen(
                        new SuperToState(superstruct, 0.5, SuperPreset.ALGAE_STOW.getState())
                    )
                ),
                new ScoreNet(superstruct, intake)
            );

        }

    }
}
