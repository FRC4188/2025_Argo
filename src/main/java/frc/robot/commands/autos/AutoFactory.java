package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.scoring.Score;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.scoring.intake.Intake;
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
        Pose2d source = AllianceFlip.flipDS(drive.getPose().nearest(Source.csources));
        List<Pose2d> avai = CoralGoal.cgoals;
        Pose2d goal = AllianceFlip.apply(drive.getPose().nearest(avai));
        return parallel(
            runOnce(() -> time.restart()),
            either(
                idle(superstructure, drive, intake),
                repeatingSequence(
                    new Score(goal, SuperState.SuperPreset.L4_CORAL.getState(), intake.eject(), drive, superstructure, intake),
                    new Score(goal, SuperState.SuperPreset.SOURCE.getState(), intake.eject(), drive, superstructure, intake),
                    Commands.runOnce(() -> avai.remove(goal))
                ), 
                ()-> time.hasElapsed(15))
        );
    }
}
