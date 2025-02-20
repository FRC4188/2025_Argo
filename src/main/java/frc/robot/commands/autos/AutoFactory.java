package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.scoring.Score;
import frc.robot.commands.scoring.AutoScore.coralScore;
import frc.robot.commands.scoring.AutoScore.coralSource;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.Intake.Mode;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Source;
import frc.robot.util.FieldConstant.Reef.CoralGoal;

import static frc.robot.util.FieldConstant.*;

import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;


public final class AutoFactory {
    static Timer time = new Timer();

    public static Command leftCoralSource(Drive drive, Superstructure superstructure, Intake intake){
        return parallel(
            runOnce(() -> time.restart()),
            either(
                idle(superstructure, drive, intake),
                repeatingSequence(
                    new coralScore(4, drive, superstructure, intake),
                    new coralSource(drive, superstructure, intake)
                ), 
                ()-> time.hasElapsed(15))
        );
    }
}
