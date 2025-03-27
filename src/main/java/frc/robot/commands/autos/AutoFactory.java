package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.scoring.AutoScore.algaeProcess;
import frc.robot.commands.scoring.AutoScore.algaeSource;
import frc.robot.commands.scoring.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef.CoralGoal;

public final class AutoFactory {
    public static Timer timer = new Timer();

    // public static Command leftL4CoralGen(Drive drive, Superstructure superstructure, Intake intake){
    //     drive.setPose(AllianceFlip.flipDS(FieldConstant.left_start));
    //     return Commands.runOnce(() -> timer.start()).andThen(
    //     Commands.repeatingSequence(
    //         new coralScore(4, drive, superstructure, intake),
    //         new coralSource(drive, superstructure, intake)
    //     ).until(() -> timer.hasElapsed(14)).andThen(
    //         new SuperToState(superstructure, SuperPreset.START.getState())
    //     ));
    // }

    // public static Command rightL4CoralGen(Drive drive, Superstructure superstructure, Intake intake){
    //     drive.setPose(AllianceFlip.flipDS(FieldConstant.right_start));
    //     return Commands.runOnce(() -> timer.start()).andThen(
    //         new coralScore(CoralGoal.right_brg_right, 4, drive, superstructure, intake)
    //         .andThen(new coralSource(drive, superstructure, intake)).andThen(
    //     Commands.repeatingSequence(
    //         new coralScore(4, drive, superstructure, intake),
    //         new coralSource(drive, superstructure, intake)
    //     ).until(() -> timer.hasElapsed(14)).andThen(
    //         new SuperToState(superstructure, SuperPreset.START.getState())
    //     )));
    // }

    // public static Command coralGen(Pose2d[] goals, Pose2d source, Pose2d starting, Drive drive, Superstructure superstructure, Intake intake){
    //     drive.setPose(starting);
    //     return Commands.runOnce(() -> timer.start()).andThen(
    //         new coralScore(goals[0], 4, drive, superstructure, intake)
    //         .andThen(Commands.parallel(
    //             new coralSource(source, drive, superstructure, intake))).andThen(
    //     Commands.repeatingSequence(
    //         new coralScore(4, drive, superstructure, intake),
    //         new coralSource(drive, superstructure, intake)
    //     ).until(() -> timer.hasElapsed(14)).andThen(
    //         new SuperToState(superstructure, SuperPreset.START.getState())
    //     )));
    // }

    /***
     * 
     * @param starting unflipped
     * @param drive
     * @param superstructure
     * @param intake
     * @return
     */
    public static Command algaeGen(Pose2d starting, Drive drive, Superstructure superstructure, Intake intake){
        return 
            Commands.parallel(
                Commands.runOnce(()-> timer.start()), 
                Commands.runOnce(()-> drive.setPose(AllianceFlip.flipDS(starting))))
            .andThen(new algaeSource(drive, superstructure, intake)
            .andThen(new algaeProcess(drive, superstructure, intake))
            .andThen(
                Commands.repeatingSequence(
                    Commands.runOnce(()-> System.out.println("sequence loop")),
                    new algaeSource(drive, superstructure, intake),
                    new algaeProcess(drive, superstructure, intake))
                .until(() -> timer.hasElapsed(14))
                .andThen(Commands.runOnce(()-> intake.eject(()->5)).until(()-> !intake.isIn()))
                .andThen(new SuperToState(superstructure, 0, SuperPreset.START.getState())
        )));
    }

    //input raw pose
    // public static Command algaeGen(Pose2d starting, Drive drive, Superstructure superstructure, Intake intake, Pose2d... goals){
        
    //     List<Pose2d> curr = Arrays.asList(goals);
    //     Timer time = new Timer();

    //     SequentialCommandGroup c = new SequentialCommandGroup(); 

    //     curr.forEach(
    //         (pose) -> 
    //             c.addCommands(
    //                 new algaeSource(pose, drive, superstructure, intake),
    //                 new algaeProcess(drive, superstructure, intake)
    //             )
    //     );
    //     return Commands
    //         .parallel(
    //             Commands.runOnce(()-> time.reset()),
    //             Commands.runOnce(()-> drive.setPose(starting))
    //         ).andThen(c).until(()-> time.hasElapsed(13))
    //         .andThen(Commands.runOnce(()-> intake.eject(()->5)).until(()-> !intake.isIn()))
    //         .andThen(new SuperToState(superstructure, 0, SuperState.SuperPreset.START.getState()));
    // }
}
