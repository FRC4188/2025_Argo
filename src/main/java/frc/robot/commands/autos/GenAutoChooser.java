package frc.robot.commands.autos;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.scoring.AutoScore;
import frc.robot.commands.scoring.AutoScore.*;
import frc.robot.commands.scoring.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Source;
import frc.robot.util.FieldConstant.Reef.AlgaeSource;
import frc.robot.util.FieldConstant.Reef.CoralGoal;

import static frc.robot.util.FieldConstant.Reef.*;

public class GenAutoChooser {
    private LoggedDashboardChooser<Pose2d> startPose = new LoggedDashboardChooser<>("Dynamic Auto/Starting Position");

    private LoggedDashboardChooser<Command> staact = new LoggedDashboardChooser<>("Dynamic Auto/Start Act");
    private LoggedDashboardChooser<Command> src1 = new LoggedDashboardChooser<>("Dynamic Auto/Src 1");
    private LoggedDashboardChooser<Command> src2 = new LoggedDashboardChooser<>("Dynamic Auto/Src 2");
    private LoggedDashboardChooser<Command> sco1 = new LoggedDashboardChooser<>("Dynamic Auto/Score 1");
    private LoggedDashboardChooser<Command> sco2 = new LoggedDashboardChooser<>("Dynamic Auto/Score 2");
    
    private double start_time = 0;

    Drive drive;
    Superstructure superstructure;
    Intake intake;

    public GenAutoChooser(Drive drive, Superstructure superstructure, Intake intake){
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;
        init();
    }

    public void init(){
        startPose.addDefaultOption("None", Pose2d.kZero);
        src1.addDefaultOption("None", Commands.none());
        src2.addDefaultOption("None", Commands.none());
        sco1.addDefaultOption("None", Commands.none());
        sco2.addDefaultOption("None", Commands.none());
        staact.addDefaultOption("None", Commands.none());

        startPose.addOption("Middle", FieldConstant.start_mid);
        startPose.addOption("Left", FieldConstant.start_left);
        startPose.addOption("Right", FieldConstant.start_right);

        sco1.addOption("Processor", new AutoScore.algaeProcess(drive, superstructure, intake));
        sco1.addOption("Net Mid", new AutoScore.algaeNet(FieldConstant.Net.mid_score, drive, superstructure, intake));
        sco1.addOption("Net Left", new AutoScore.algaeNet(FieldConstant.Net.left_score, drive, superstructure, intake));
        sco1.addOption("Net Right", new AutoScore.algaeNet(FieldConstant.Net.right_score, drive, superstructure, intake));
        sco1.addOption("Net Closest", new AutoScore.algaeNet(drive, superstructure, intake));

        src1.addOption("Src Brg", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.mid_brg_src, drive, superstructure, intake));
        src1.addOption("Src Brg Right", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.right_brg_src, drive, superstructure, intake));
        src1.addOption("Src Brg Left", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.left_brg_src, drive, superstructure, intake));
        src1.addOption("Src Ally", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.alliance_src, drive, superstructure, intake));
        src1.addOption("Src Ally Right", new AutoScore.algaeNet(FieldConstant.Reef.AlgaeSource.right_src_src, drive, superstructure, intake));
        src1.addOption("Src Ally Left", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.left_src_src, drive, superstructure, intake));
        src1.addOption("Src Closest", new AutoScore.algaeSource(drive, superstructure, intake));


        sco2.addOption("Processor", new AutoScore.algaeProcess(drive, superstructure, intake));
        sco2.addOption("Net Mid", new AutoScore.algaeNet(FieldConstant.Net.mid_score, drive, superstructure, intake));
        sco2.addOption("Net Left", new AutoScore.algaeNet(FieldConstant.Net.left_score, drive, superstructure, intake));
        sco2.addOption("Net Right", new AutoScore.algaeNet(FieldConstant.Net.right_score, drive, superstructure, intake));
        sco2.addOption("Net Closest", new AutoScore.algaeNet(drive, superstructure, intake));

        src2.addOption("Src Brg", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.mid_brg_src, drive, superstructure, intake));
        src2.addOption("Src Brg Right", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.right_brg_src, drive, superstructure, intake));
        src2.addOption("Src Brg Left", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.left_brg_src, drive, superstructure, intake));
        src2.addOption("Src Ally", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.alliance_src, drive, superstructure, intake));
        src2.addOption("Src Ally Right", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.right_src_src, drive, superstructure, intake));
        src2.addOption("Src Ally Left", new AutoScore.algaeSource(FieldConstant.Reef.AlgaeSource.left_src_src, drive, superstructure, intake));
        src2.addOption("Src Closest", new AutoScore.algaeSource(drive, superstructure, intake));

        staact.addOption("Push Left", AutoScore.pushLeave(drive).andThen(new DriveTo(drive, FieldConstant.start_left)));
        staact.addOption("Push Right", AutoScore.pushLeave(drive).andThen(new DriveTo(drive, FieldConstant.start_right)));
    }

    public Command getAutonomousCommand(){
        return Commands.sequence(
            AutoTests.init(startPose.get(), drive, superstructure),
            Commands.runOnce(() -> start_time = Timer.getFPGATimestamp()),
            staact.get(),
            src1.get(),
            sco1.get(),
            src2.get(),
            sco2.get()
            ).until(()-> Timer.getFPGATimestamp() - start_time> 14)
            .andThen(
                Commands.parallel(
                    new DriveTo(drive, startPose.get()),
                    new SuperToState(superstructure, 0.5, SuperState.SuperPreset.ALGAE_STOW.getState())
                )
            );
    }

}
