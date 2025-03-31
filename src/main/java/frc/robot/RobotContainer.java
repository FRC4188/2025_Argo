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
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.autos.AutoFactory;
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.autos.GenAutoChooser;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.scoring.ScoreNet;
import frc.robot.commands.scoring.SuperToState;
import frc.robot.commands.scoring.AutoScore;
import frc.robot.commands.scoring.AutoScore.algaeNet;
import frc.robot.inputs.CSP_Controller;
import frc.robot.inputs.CSP_Controller.Scale;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXReal;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOPigeon2;
import frc.robot.subsystems.scoring.climber.Climber;
import frc.robot.subsystems.scoring.climber.ClimberIOReal;
import frc.robot.subsystems.scoring.climber.ClimberIOSim;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.IntakeIO;
import frc.robot.subsystems.scoring.intake.IntakeIOReal;
import frc.robot.subsystems.scoring.intake.IntakeIOSim;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperVisualizer;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.subsystems.vision.VisionIOLL;
import frc.robot.util.FieldConstant;
import frc.robot.util.FieldConstant.Reef.AlgaeSource;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
  private Limelight vis;
  //sim
  private SwerveDriveSimulation driveSim;
  private SuperVisualizer supervis;

  private Runnable resetGyro;

  // Controller
  private final CSP_Controller controller = new CSP_Controller(0);
  private final CSP_Controller controller2 = new CSP_Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final GenAutoChooser genChooser;

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
        vis = 
            new Limelight(
              drive,
              new VisionIOLL(VisConstants.frontLL, ()-> new Rotation2d())
            );
        superstructure = new Superstructure(Mode.REAL);


        intake = new Intake(new IntakeIOReal());
        break;

      case SIM:
        //maple sim
        
        // Sim robot, instantiate physics sim IO implementations
        driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(7.459, 5.991, Rotation2d.k180deg));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        drive =
            new Drive(
                // new GyroIOSim(driveSim.getGyroSimulation()),
                new GyroIO() {},
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft),
                new ModuleIOTalonFXSim(TunerConstants.BackRight),
                driveSim::setSimulationWorldPose);
        
        

        // vis = new Limelight(drive, new VisionIO(){});
        superstructure = new Superstructure(Mode.SIM);

        supervis = new SuperVisualizer("Models", 
        () -> superstructure.getEleHeight(),
        () -> superstructure.getWristAngle(), 
        () -> 0);

        intake = new Intake(new IntakeIOSim());
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
        superstructure = new Superstructure(Mode.SIM);
        break;
    }
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    genChooser = new GenAutoChooser(drive, superstructure, intake);
    resetGyro = 
    Constants.robot.currMode == Constants.Mode.SIM? 
      () -> drive.setPose(
                driveSim
                  .getSimulatedDriveTrainPose()): // reset odometry to actual robot pose during simulation

       () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    //add cmds for pathplanner events
    HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(            
            Map.entry("Delay", new WaitCommand(1.5))
            // Map.entry("Super Start", 
            //   new SuperToState(superstructure, SuperPreset.START.getState())),
            // Map.entry("Coral L3", 
            //   new SuperToState(superstructure, SuperPreset.L3_CORAL.getState())),
            // Map.entry("Coral L4", 
            //   new SuperToState(superstructure, SuperPreset.L4_CORAL.getState())),
            // Map.entry("Coral L2", 
            //   new SuperToState(superstructure, SuperPreset.L3_CORAL.getState())),
            // Map.entry("Coral Source", 
            //   new SuperToState(superstructure, SuperPreset.SOURCE_REVERSE.getState())),
            // Map.entry("Algae L3", 
            //   new SuperToState(superstructure, SuperPreset.L3_ALGAE.getState())),
            // Map.entry("Algae L2", 
            //   new SuperToState(superstructure, SuperPreset.L2_ALGAE.getState()))
            // Map.entry("Score Coral", 
            //   Commands.run(()-> 
            //     intake.ingest(Intake.Mode.ALGAE, false)).withTimeout(1)
            //   .andThen(Commands.run(()-> intake.stop()))),
            // Map.entry("Score Algae", 
            //   Commands.run(()-> 
            //     intake.ingest(Intake.Mode.CORAL, false)).withTimeout(1)
            //   .andThen(Commands.run(()-> intake.stop()))),
            // Map.entry("Get Coral", 
            //   Commands.run(()-> 
            //     intake.ingest(Intake.Mode.CORAL, false)).withTimeout(2.5)
            //   .andThen(Commands.run(()-> intake.stop()))),
            // Map.entry("Get Algae", 
            //   Commands.run(()-> 
            //     intake.ingest(Intake.Mode.ALGAE, false)).withTimeout(2.5)
            //   .andThen(Commands.run(()-> intake.stop())))
        )
      );

    NamedCommands.registerCommands(EVENTS);

    configureDashboard();
    configureButtonBindings();
    teleInit();
  }

  public void teleInit() {
    superstructure.setTarget(new SuperState());
    superstructure.resetEle();
    drive.setPose();
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
    superstructure.setDefaultCommand(Commands.run(superstructure::disable_manual, superstructure));

    Trigger drivingInput = new Trigger(() -> (controller.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0 || controller.getCorrectedRight(Scale.LINEAR).getX() != 0.0));

    drivingInput.onTrue(DriveCommands.TeleDrive(drive,
      () -> -controller.getCorrectedLeft(Scale.LINEAR).getY() * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0),
      () -> -controller.getCorrectedLeft(Scale.LINEAR).getX() * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0),
      () -> -controller.getCorrectedRight(Scale.SQUARED).getX() * (controller.getRightBumperButton().getAsBoolean() ? 0.25 : 1.0)));

    Trigger superInput = new Trigger(() -> (controller2.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0 || controller2.getCorrectedRight(Scale.LINEAR).getNorm() != 0.0));

    superInput.onTrue(superstructure.manual(
      () -> controller2.getCorrectedLeft(Scale.LINEAR).getY(),
      () -> controller2.getCorrectedRight(Scale.LINEAR).getY()
    ));

    // Reset gyro to 0° when start button is pressed
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true)); 
      
    controller.getLeftTButton().whileTrue(
      intake.ingest(()-> ((controller.getLeftTriggerAxis() >= 0.75)?10:4)))
        .onFalse(intake.stop());
      
    controller.getRightTButton().whileTrue(
      intake.eject(()-> controller.getRightTriggerAxis() * 8))
        .onFalse(intake.stop());

    //emergency cases
    controller2.x().and(controller2.leftBumper()).onTrue(superstructure.resetEle());
    controller2.b().and(controller2.leftBumper()).onTrue(Commands.runOnce(() -> superstructure.wrist_pid = !superstructure.wrist_pid));
    controller2.y().and(controller2.leftBumper()).onTrue(Commands.runOnce(() -> superstructure.ele_pid = !superstructure.ele_pid));
    controller2.a().onTrue(Commands.runOnce(() -> drive.vision_accept = !drive.vision_accept));

    controller2.getStartButton().onTrue(new SuperToState(superstructure, 0, SuperPreset.START.getState()));

    controller2.getRightBumperButton().onTrue(
      new ScoreNet(superstructure, intake));
    controller2.getUpButton().onTrue(
      new SuperToState(superstructure, 0.5, SuperPreset.PROCESSOR.getState()));

    controller2.getLeftButton().onTrue(
      new SuperToState(superstructure, 0, SuperPreset.L2_ALGAE.getState()));

    controller2.getRightButton().onTrue(
      new SuperToState(superstructure, 0, SuperPreset.L3_ALGAE.getState()));
      
    controller2.getDownButton().onTrue(
      new SuperToState(superstructure, SuperPreset.ALGAE_GROUND.getState().getWristAngle()));
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
    autoChooser.addDefaultOption(
        "Drive SysId (Dynamic Forward)", 
        Commands.runOnce(() -> superstructure.resetEle()).andThen(drive.sysIdDynamic(SysIdRoutine.Direction.kForward)));

    autoChooser.addOption("gui 3 left source", new PathPlannerAuto("3 Left Corals"));
    autoChooser.addOption("gui 3 right source", new PathPlannerAuto("3 Right Corals"));
    autoChooser.addOption("test leave ", 
      Commands.sequence(
        AutoTests.init(FieldConstant.start_left, drive, superstructure),
        AutoScore.pushLeave(drive)));

    autoChooser.addOption("test source", 
      Commands.sequence(
        AutoTests.init(FieldConstant.start_left, drive, superstructure),
        new AutoScore.algaeSource(drive, superstructure, intake)));
    
    autoChooser.addOption("test process", 
      Commands.sequence(
        AutoTests.init(FieldConstant.start_left, drive, superstructure),
        new AutoScore.algaeSource(drive, superstructure, intake),
        new AutoScore.algaeProcess(drive, superstructure, intake),
        new AutoScore.algaeSource(drive, superstructure, intake),
        new AutoScore.algaeProcess(drive, superstructure, intake)
      ));

    autoChooser.addOption("test net", 
      Commands.sequence(
          AutoTests.init(FieldConstant.start_left, drive, superstructure),
          new AutoScore.algaeSource(drive, superstructure, intake),
          new AutoScore.algaeNet(drive, superstructure, intake),
          new AutoScore.algaeSource(drive, superstructure, intake)));

    autoChooser.addOption("descore", 
      Commands.sequence(
          AutoTests.init(FieldConstant.start_left, drive, superstructure),
          new AutoScore.algaeSource(drive, superstructure, intake),
          new AutoScore.descore(drive, superstructure, intake),
          new AutoScore.algaeSource(drive, superstructure, intake),
          new AutoScore.descore(drive, superstructure, intake)));
    
    autoChooser.addOption("test coral", 
      Commands.sequence(
          AutoTests.init(FieldConstant.start_left, drive, superstructure),
          new AutoScore.coralScore(drive, superstructure, intake)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     return autoChooser.get();
    // return genChooser.getAutonomousCommand();
  }

  public void resetSimulation(){
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(0, 0, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    superstructure.setTarget(new SuperState());
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.robot.currMode != Constants.Mode.SIM) return;

    supervis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", driveSim.getSimulatedDriveTrainPose());

    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}