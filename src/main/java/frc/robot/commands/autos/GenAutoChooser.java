package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.scoring.AutoScore.*;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant.Source;
import frc.robot.util.FieldConstant.Reef.AlgaeSource;
import frc.robot.util.FieldConstant.Reef.CoralGoal;

import static frc.robot.util.FieldConstant.Reef.*;

public class GenAutoChooser {
    private final LoggedDashboardChooser<Pose2d> startPose = new LoggedDashboardChooser<>("Dynamic Auto/Starting Position");
    // private final LoggedDashboardChooser<Pose2d> source = new LoggedDashboardChooser<>("Dynamic Auto/Main source");
    private final static LoggedNetworkString goal = new LoggedNetworkString("Dynamic Auto/Goals");
    static List<Pair<Pose2d, SuperPreset>> routine = new ArrayList<>();
    static List<Command> cmds = new ArrayList<>();
    
    public static GenAutoChooser instance;
    public static final GenAutoChooser getInstance(){
        if(instance == null) instance = new GenAutoChooser();
        return instance;
    }

    private GenAutoChooser(){
        init();
    }

    public void init(){
        startPose.addOption("Middle", new Pose2d(7.180, 4.019563674926758, Rotation2d.k180deg));
        startPose.addOption("Left", new Pose2d(7.180, 7.550, Rotation2d.k180deg));
        startPose.addOption("Right", new Pose2d(7.180, 0.480, Rotation2d.k180deg));
        startPose.addOption("None", new Pose2d());
                            
        // source.addOption("Left_Far", AllianceFlip.flipDS(Source.left_src_far));
        // source.addOption("Left_Close", AllianceFlip.flipDS(Source.left_src_close));
        // source.addOption("Left_Mid", AllianceFlip.flipDS(Source.left_src_mid));
        // source.addOption("Right_Far", AllianceFlip.flipDS(Source.right_src_far));
        // source.addOption("Right_Close", AllianceFlip.flipDS(Source.right_src_close));
        // source.addOption("Right_Mid", AllianceFlip.flipDS(Source.right_src_mid));
                                    
    }
                            
    // public Command getAutonomousCommand(Drive drive, Superstructure superstructure, Intake intake){
    //     setRoutine(drive, superstructure, intake);
    //     return Commands.repeatingSequence(
    //         cmds.get(0),
    //         Commands.runOnce(()-> cmds.remove(0))
    //     ).until(()-> routine.size() == 0);
    // }
                            
    // private void setRoutine(Drive drive, Superstructure superstructure, Intake intake){
    //     String[] ree = goal.get().split(" ");

    //     cmds.add(
    //         new algaeSource(
    //             getAlgaePos(ree[0].charAt(2), ree[0].charAt(1)), 
    //             drive, superstructure, intake)
    //     );
                        
    //     //close/far -> location
    //     for(int i = 1; i < ree.length; i ++){
    //         cmds.add(new algaeProcess(drive, superstructure, intake));
    //         cmds.add(new algaeSource(
    //             getAlgaePos(ree[i].charAt(2), ree[i].charAt(1)), 
    //             drive, superstructure, intake));
    //     }
        

    // }

    // private Pose2d getAlgaePos(char loc, char far){
    //     switch(loc){ // which 
    //         case 'a':
    //             return AllianceFlip.flipDS(isFar(far)? AlgaeSource.left_brg_src: AlgaeSource.left_src_src);
    //         case 'b':
    //             return AllianceFlip.flipDS(isFar(far)? AlgaeSource.mid_brg_src: AlgaeSource.alliance_src);
    //         case 'c':
    //             return AllianceFlip.flipDS(isFar(far)? AlgaeSource.right_brg_src: AlgaeSource.right_src_src);
    //     }
    //     return new Pose2d();
    // }

    private boolean isFar(char far){
        return far == 'c'? true: false;
    }

    // private Pose2d getCoralGoalPos(char loc, char far){
    //     switch(loc){ // which 
    //         case 'a':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.left_brg_left: CoralGoal.left_src_left);

    //         case 'b':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.left_brg_right: CoralGoal.left_src_right);

    //         case 'c':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.mid_brg_left: CoralGoal.alliance_left);

    //         case 'd':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.mid_brg_right: CoralGoal.alliance_right);

    //         case 'e':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.right_brg_left: CoralGoal.right_src_left);

    //         case 'f':
    //             return AllianceFlip.flipDS(isFar(far)? CoralGoal.right_brg_right: CoralGoal.right_src_right);
    //     }
    //     return new Pose2d();
    // }


}
