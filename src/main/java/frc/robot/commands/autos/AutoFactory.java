package frc.robot.commands.autos;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.scoring.AutoScore.coralScore;
import frc.robot.commands.scoring.AutoScore.coralSource;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.FieldConstant.Reef.CoralGoal;

public final class AutoFactory {
    public static Timer timer = new Timer();

    public static Command leftL4CoralGen(Drive drive, Superstructure superstructure, Intake intake){
        return Commands.runOnce(() -> timer.start()).andThen(
        Commands.repeatingSequence(
            new coralScore(4, drive, superstructure, intake),
            new coralSource(drive, superstructure, intake)
        ));//.until(() -> timer.hasElapsed(15)));
    }

    public static Command rightL4CoralGen(Drive drive, Superstructure superstructure, Intake intake){

        return Commands.runOnce(() -> timer.start()).andThen(
            new coralScore(CoralGoal.right_brg_right, 4, drive, superstructure, intake)
            .andThen(new coralSource(drive, superstructure, intake)).andThen(
        Commands.repeatingSequence(
            new coralScore(4, drive, superstructure, intake),
            new coralSource(drive, superstructure, intake)
        ).until(() -> timer.hasElapsed(15))));
    }
}
