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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.autos.AutoFactory;
import choreo.auto.*;
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.superstructure.SuperToState;
import frc.robot.inputs.CSP_Controller;
import frc.robot.inputs.CSP_Controller.Scale;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXReal;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.IntakeIO;
import frc.robot.subsystems.scoring.intake.IntakeIOReal;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperVisualizer;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLL;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Source;

import java.util.HashMap;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Superstructure superstructure;
  private Intake intake;

  // private final Limelight vis;
  private SwerveDriveSimulation driveSim;
  private  Runnable resetGyro;

  // Controller
  private final CSP_Controller controller = new CSP_Controller(0);
  private final CSP_Controller controller2 = new CSP_Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  @AutoLogOutput(key  = "Copilot/IntakeMode")
  private Intake.Mode currMode = Intake.Mode.CORAL;

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
        
        // vis = new Limelight(drive, 
        //     new VisionIOLL("limelight-back", drive::getRotation));        
            superstructure = new Superstructure(Mode.REAL);

        intake = new Intake(new IntakeIOReal());
        break;

      case SIM:
        //maple sim
        
        // Sim robot, instantiate physics sim IO implementations
        // driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig,new Pose2d(8.251, 5.991, new Rotation2d(Degrees.of(-178.059))));
        driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(7.459, 5.991, Rotation2d.k180deg));
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

        // vis = new Limelight(drive, new VisionIO(){});

        superstructure = new Superstructure(Mode.SIM);
        intake = new Intake(new IntakeIO() {});
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

        // vis = new Limelight(drive, new VisionIO(){});
        break;
    }
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    resetGyro = Constants.robot.currMode == Constants.Mode.SIM
      ? () -> drive.setPose(
                driveSim
                  .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation

      : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    //add cmds for pathplanner events
    HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(            
            Map.entry("Delay", new WaitCommand(1.5)),
            Map.entry("Super Start", 
              new SuperToState(superstructure, SuperPreset.START.getState())),
            Map.entry("Coral L3", 
              new SuperToState(superstructure, SuperPreset.L3_CORAL.getState())),
            Map.entry("Coral L4", 
              new SuperToState(superstructure, SuperPreset.L4_CORAL.getState())),
            Map.entry("Coral L2", 
              new SuperToState(superstructure, SuperPreset.L3_CORAL.getState())),
            Map.entry("Coral Source", 
              new SuperToState(superstructure, SuperPreset.SOURCE_REVERSE.getState())),
            Map.entry("Algae L3", 
              new SuperToState(superstructure, SuperPreset.L3_ALGAE.getState())),
            Map.entry("Algae L2", 
              new SuperToState(superstructure, SuperPreset.L2_ALGAE.getState())),
            Map.entry("Score Coral", 
              Commands.run(()-> 
                intake.ingest(Intake.Mode.ALGAE)).withTimeout(1)
              .andThen(Commands.run(()-> intake.stop()))),
            Map.entry("Score Algae", 
              Commands.run(()-> 
                intake.ingest(Intake.Mode.CORAL)).withTimeout(1)
              .andThen(Commands.run(()-> intake.stop()))),
            Map.entry("Get Coral", 
              Commands.run(()-> 
                intake.ingest(Intake.Mode.CORAL)).withTimeout(2.5)
              .andThen(Commands.run(()-> intake.stop()))),
            Map.entry("Get Algae", 
              Commands.run(()-> 
                intake.ingest(Intake.Mode.ALGAE)).withTimeout(2.5)
              .andThen(Commands.run(()-> intake.stop())))
        )
      );

    NamedCommands.registerCommands(EVENTS);

    configureDashboard();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(Commands.runOnce(drive::stopWithX, drive));

    superstructure.setDefaultCommand(
      Commands.run(() -> superstructure.manualOverride(
        () -> 3 * -controller2.getLeftY(), 
        () -> 3 * -controller2.getRightY(), 
        () -> 3 * ( controller2.getRightT(Scale.SQUARED) - controller2.getLeftT(Scale.SQUARED))), superstructure
        , superstructure));
    

    Trigger drivingInput = new Trigger(() -> (controller.getCorrectedLeft().getNorm() != 0.0 || controller.getCorrectedRight().getX() != 0.0));

    drivingInput.onTrue(DriveCommands.TeleDrive(drive,
      () -> -controller.getCorrectedLeft().getX() * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0),
      () -> -controller.getCorrectedLeft().getY() * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0),
      () -> controller.getRightX(Scale.SQUARED) * 0.7 * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0)));

    // Reset gyro to 0° when start button is pressed
      
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true)); 
      
    // controller.a().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    //outtake
    controller.getLeftTButton().onTrue(
        intake.ingest((currMode == Intake.Mode.CORAL)? Intake.Mode.ALGAE:Intake.Mode.CORAL)
    ).onFalse(intake.stop());

    //intake
    controller.getRightTButton().onTrue(
      intake.ingest((currMode))
    ).onFalse(intake.stop());

    // controller.a().onTrue(
    //   new DriveToPose(drive, ()-> drive.getPose().nearest(Source.csources))
    // ).onFalse(
    //   Commands.runOnce(()-> drive.stop())
    // );

      //manual controls down here
    controller2.a().onTrue(Commands.runOnce(() -> superstructure.armOverride = !superstructure.armOverride));

    controller2.b().onTrue(Commands.runOnce(() -> superstructure.wristOverride = !superstructure.wristOverride));

    controller2.y().onTrue(Commands.runOnce(() -> superstructure.eleOverride = !superstructure.eleOverride));

    controller2.x().onTrue(superstructure.resetEle());

    controller2.getRightBumperButton().onTrue(
      Commands.runOnce(() -> currMode = currMode == Intake.Mode.CORAL? Intake.Mode.ALGAE: Intake.Mode.CORAL));

    controller2.getStartButton().onTrue(new SuperToState(superstructure, SuperPreset.START.getState()));

    controller2.getUpButton().onTrue(
      new ConditionalCommand(
        new SuperToState(superstructure, SuperPreset.SOURCE.getState()), 
        new SuperToState(superstructure, SuperPreset.PROCESSOR_REVERSE.getState()), 
        () -> currMode == Intake.Mode.CORAL));

    controller2.getLeftButton().onTrue(
      new ConditionalCommand(
        new SuperToState(superstructure, SuperPreset.L2_CORAL.getState()), 
        new SuperToState(superstructure, SuperPreset.L2_ALGAE_REVERSE.getState()),
        ()-> currMode == Intake.Mode.CORAL));
    
    controller2.getRightButton().onTrue(
      new ConditionalCommand(
        new SuperToState(superstructure, SuperPreset.L3_CORAL.getState()), 
        new SuperToState(superstructure, SuperPreset.L3_ALGAE_REVERSE.getState()),
        ()-> currMode == Intake.Mode.CORAL));
      
    controller2.getDownButton().onTrue(
      new ConditionalCommand(
      new SuperToState(superstructure, SuperPreset.L4_CORAL.getState()), 
      new SuperToState(superstructure, SuperPreset.ALGAE_GROUND.getState()),
      ()-> currMode == Intake.Mode.CORAL));;

  }

  private void configureDashboard() {

    
    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        //pathplanner pathfinding + following
    // autoChooser.addOption("Mid to 2 corals gui", AutoTests.twoCoral());
    autoChooser.addOption("left source coral ", AutoFactory.leftL4CoralGen(drive, superstructure, intake));
    autoChooser.addOption("right source coral", AutoFactory.rightL4CoralGen(drive, superstructure, intake));

    //drive to pose cmmd test
    autoChooser.addOption("2 corals drive", AutoTests.drive2Corals(drive));
    autoChooser.addOption("pathgen", AutoTests.AG2Coral(drive));

    autoChooser.addOption("left source coral", AutoFactory.leftL4CoralGen(drive, superstructure, intake));
    autoChooser.addOption("right source coral", AutoFactory.rightL4CoralGen(drive, superstructure, intake));
    autoChooser.addOption("algae middle", AutoFactory.algaeGen(FieldConstant.mid_start, drive, superstructure, intake));
    autoChooser.addOption("algae left", AutoFactory.algaeGen(FieldConstant.left_start, drive, superstructure, intake));
    autoChooser.addOption("algae Right", AutoFactory.algaeGen(FieldConstant.right_start, drive, superstructure, intake));

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
    superstructure.resetEle();
    superstructure.resetWrist();
    superstructure.setTarget(new SuperState(0, 0, 0));
    return autoChooser.get();
  }

  public void resetSimulation(){
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(7.459, 5.991, new Rotation2d()));
    // drive.setPose(new Pose2d(0, 0, new Rotation2d(Degrees.of(0))));
    //drive.setPose(FieldConstant.Source.left_src_mid);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});

    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}