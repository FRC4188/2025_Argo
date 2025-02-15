// package frc.robot.commands.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.commands.drive.DriveTo;
// import frc.robot.subsystems.drivetrain.Drive;
// import frc.robot.subsystems.generated.TunerConstants;
// import frc.robot.subsystems.scoring.Superstructure;
// import frc.robot.util.AllianceFlip;
// import frc.robot.util.FieldConstant.Reef;
// import frc.robot.util.FieldConstant.Source;

// import static frc.robot.util.FieldConstant.*;
// import static edu.wpi.first.wpilibj2.command.Commands.*;


// public final class AutoFactory {
//     static Timer time = new Timer();
//     static TrajectoryConfig config = new TrajectoryConfig(
//         TunerConstants.kSpeedAt12Volts.magnitude() / 2, 
//         Constants.robot.MAX_ACCELERATION.magnitude() / 2);

//     public static Command leftCoralSource(Drive drive, Superstructure superstructure){
//         Pose2d source = AllianceFlip.flipDS(Source.left_srcs[2]);
//         return parallel(
//             runOnce(() -> time.restart()),
//             either(
//                 idle(superstructure),
//                 repeatingSequence(
//                     ()->{
//                         int c = 0;
                        
//                     }
//                 ), 
//                 ()-> time.hasElapsed(15))
//         );
//     }
// }
