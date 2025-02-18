package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.robot;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.superstructure.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.FieldConstant;

public class AutoScore extends SequentialCommandGroup {
    public double happy_zone = 2;

    public AutoScore(Pose2d goal, SuperState state, Drive drive, Superstructure superstructure, Intake intake) {

        addCommands(
            new DriveTo(drive, goal).alongWith(
                new WaitUntilCommand(() -> (drive.getPose().getTranslation().getDistance(goal.getTranslation()) <= happy_zone))
                .andThen(new SuperToState(superstructure, state))),
            intake.eject()
        );

    }
    
}
