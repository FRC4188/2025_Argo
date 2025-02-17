// package frc.robot.commands.drive;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.autos.AutoTests;
// import frc.robot.subsystems.drivetrain.Drive;
// import frc.robot.subsystems.vision.Limelight;

// public class DriveToTag extends SequentialCommandGroup {
//     public DriveToTag(Drive drive, String llName){
//         addCommands(
//             new DriveTo(drive, Limelight.targetID(drive), AutoTests.config)
//             .andThen(
//                 new TagTracking(drive, llName)
//         ));
//     }
// }
