package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.superstructure.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.Intake.Mode;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
public class Score extends SequentialCommandGroup {
    public double happy_zone = 3;

    public Score(Pose2d goal, SuperState state, Command intakeCommand, Drive drive, Superstructure superstructure, Intake intake) {


        addCommands(
            new DriveTo(drive, goal).alongWith(
                new SuperToState(superstructure, (Intake.intakeState == Mode.ALGAE)?SuperPreset.L2_ALGAE.getState():SuperPreset.START.getState())
                .until(() -> (drive.getPose().getTranslation().getDistance(goal.getTranslation()) <= happy_zone))
                .andThen(new WaitUntilCommand(() -> (drive.getPose().getTranslation().getDistance(goal.getTranslation()) <= happy_zone))
                .andThen(new SuperToState(superstructure, state)))),
            intakeCommand.withTimeout(1)
        );

    }
    
}
