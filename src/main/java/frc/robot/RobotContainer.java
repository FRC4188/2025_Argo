// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.FollowPath;
import frc.robot.inputs.CSP_Controller;
import frc.robot.inputs.CSP_Controller.Scale;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXReal;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.scoring.SuperVisualizer;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLL;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Source;
import frc.robot.util.FieldConstant.Reef.Base.*;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Limelight vis;
  private SwerveDriveSimulation driveSim = null;
  private SuperVisualizer armSim;

  // Controller
  private final CSP_Controller controller = new CSP_Controller(0);
  private final CSP_Controller controller2 = new CSP_Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.robot.currMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        
        vis = new Limelight(drive, 
            new VisionIOLL("limelight-back", drive::getRotation),
            new VisionIOLL("limelight-front", drive::getRotation));

        break;

      case SIM:
        //maple sim
        
        // Sim robot, instantiate physics sim IO implementations
        driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig,new Pose2d(8.251, 5.991, new Rotation2d(Degrees.of(-178.059))));
        // driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig,new Pose2d(0, 0, new Rotation2d(Degrees.of(0))));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        drive =
            new Drive(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                  TunerConstants.FrontLeft, driveSim.getModules()[0]),
                new ModuleIOTalonFXSim(
                  TunerConstants.FrontRight, driveSim.getModules()[1]),
                new ModuleIOTalonFXSim(
                  TunerConstants.BackLeft, driveSim.getModules()[2]),
                new ModuleIOTalonFXSim(
                  TunerConstants.BackRight, driveSim.getModules()[3]),
                driveSim::setSimulationWorldPose);

        vis = new Limelight(drive, new VisionIO(){}, new VisionIO(){});

        armSim = new SuperVisualizer("Superstructure");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});

        vis = new Limelight(drive, new VisionIO(){}, new VisionIO(){});
        break;
    }
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureDashboard();
    // Configure the button bindings
    configureButtonBindings();

    NamedCommands.registerCommands(AutoTests.EVENTS);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        Commands.runOnce(drive::stopWithX, drive));

    Trigger drivingInput = new Trigger(() -> (controller.getCorrectedLeft().getNorm() != 0.0 || controller.getCorrectedRight().getX() != 0.0));


    // drivingInput.onTrue(DriveCommands.TeleDrive(drive,
    //   () -> -controller.getCorrectedLeft().getX() * 3.0 * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
    //   () -> -controller.getCorrectedLeft().getY() * 3.0 * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
    //   () -> (controller.getRightX(Scale.SQUARED) * 3.5 * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0))));

    drivingInput.onTrue(DriveCommands.TeleDrive(drive,
      () -> -controller.getLeftY() * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
      () -> -controller.getLeftX() * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
      () -> (controller.getRightX() * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0))));

    // Reset gyro to 0° when start button is pressed
    final Runnable resetGyro = Constants.robot.currMode == Constants.Mode.SIM
      ? () -> drive.setPose(
              driveSim
                      .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
      : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
      
      controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true)); 
      
      
  }

  private void configureDashboard() {
    // Set up auto routines

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    //pathplanner pathfinding + following
    autoChooser.addOption("Mid to 2 corals manual", AutoTests.toBasetoSource());
    autoChooser.addOption("Mid to 2 corals gui", AutoTests.twoCoral());

    //follow path commd test
    autoChooser.addOption("2 corals manual follow", AutoTests.follow2Coral(drive));

    //drive to pose cmmd test
    autoChooser.addOption("2 corals drive", AutoTests.drive2Corals(drive));
    autoChooser.addOption("pathgen", AutoTests.AG2Coral(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoTests.AG2Coral(drive);
  }

  public void resetSimulation(){
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    //drive.setPose(new Pose2d(8.251, 5.991, new Rotation2d(Degrees.of(-178.059))));
    drive.setPose(new Pose2d(0, 0, new Rotation2d(Degrees.of(0))));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});
    Logger.recordOutput("FinalComponentPoses", 
      new Pose3d[] {
        new Pose3d(
          0.127, 0.356, 0.08, new Rotation3d(0,0,-90)
        ),
        new Pose3d(
          0.174, 0.153, 0.157, new Rotation3d(0,0,0)
        ),
        new Pose3d(
          0.183, 0.177, 0.227, new Rotation3d(25,0,0)
        ),
        new Pose3d(
          0.045, 0.103, 0.65, new Rotation3d(90,0,0)
        )
      }
    );
    armSim.update(0, 0, 0);
    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}