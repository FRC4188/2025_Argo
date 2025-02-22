package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.scoring.AutoScore.coralScore;
import frc.robot.commands.scoring.AutoScore.coralSource;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.FieldConstant.Reef;

public final class AutoFactory {
    public static Timer timer = new Timer();
    public static Command leftCoralSource(Drive drive, Superstructure superstructure, Intake intake){
        // List<Pose2d> avai = Reef.CoralGoal.cgoals;
        // Collections.sort(avai, Comparator.comparingDouble((Pose2d a)-> drive.getPose().getTranslation().getDistance(a.getTranslation())));
        return Commands.runOnce(() -> timer.start()).andThen(
            Commands.repeatingSequence(
                new coralScore(4, drive, superstructure, intake),
                new coralSource(drive, superstructure, intake))
                // parallel(
                //     new InstantCommand(() -> avai.remove(0))))
            .until(() -> timer.hasElapsed(15)));
    }
}
