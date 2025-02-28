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

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.autos.AutoFactory;
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.autos.GenAutoChooser;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.superstructure.SuperToState;
import frc.robot.inputs.CSP_Controller;
import frc.robot.inputs.CSP_Controller.Scale;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOSim;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.IntakeIO;
import frc.robot.subsystems.scoring.intake.IntakeIOReal;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLL;
import frc.robot.util.FieldConstant;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.commands.scoring.AutoScore.*;

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
  private Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.magnitude());

  private final Limelight vis;

  @AutoLogOutput(key = "Current Score Level")
  private int level = 4;


  // Controller
  private final CSP_Controller controller = new CSP_Controller(0);
  private final CSP_Controller controller2 = new CSP_Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = TunerConstants.createDrivetrain();

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
            new VisionIOLL("limelight-back", drive::getRotation));        
            superstructure = new Superstructure(Mode.REAL);

        intake = new Intake(new IntakeIOReal());
        break;

      case SIM:
        

        vis = new Limelight(drive, new VisionIO(){}, new VisionIO(){});

        superstructure = new Superstructure(Mode.SIM);
        intake = new Intake(new IntakeIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
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
  public void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(Commands.runOnce(drive::stopWithX, drive));

    superstructure.setDefaultCommand(
      Commands.run(() -> superstructure.manualOverride(
        () -> controller2.getLeftY(), 
        () -> controller2.getRightY(), 
        () -> 3* controller2.getRightT(Scale.SQUARED) - controller2.getLeftT(Scale.SQUARED)), superstructure
        ));
    

    Trigger drivingInput = new Trigger(() -> (controller.getCorrectedLeft().getNorm() != 0.0 || controller.getCorrectedRight().getX() != 0.0));

    drivingInput.onTrue(DriveCommands.TeleDrive(drive,
      () -> controller.getCorrectedLeft().getX() * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
      () -> controller.getCorrectedLeft().getY() * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0),
      () -> controller.getRightX(Scale.SQUARED) * (controller.getRightBumperButton().getAsBoolean() ? 0.5 : 1.0)));

    // Reset gyro to 0° when start button is pressed
    Runnable resetGyro = Constants.robot.currMode == Constants.Mode.SIM
      ? () -> drive.setPose(
                driveSim
                  .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation

      : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
      
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true)); 
      
    // controller.a().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // controller.b().onTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // controller.x().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // controller.y().onTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    controller.leftTrigger().onTrue(
      intake.ingest(Intake.Mode.ALGAE)
    ).onFalse(intake.stop());

    controller.rightTrigger().onTrue(
      intake.ingest(Intake.Mode.CORAL)
    ).onFalse(intake.stop());

      //manual controls down here
    controller2.a().onTrue(Commands.runOnce(() -> superstructure.armOverride = !superstructure.armOverride));

    controller2.b().onTrue(Commands.runOnce(() -> superstructure.wristOverride = !superstructure.wristOverride));

    controller2.y().onTrue(Commands.runOnce(() -> superstructure.eleOverride = !superstructure.eleOverride));

      controller2.x().onTrue(superstructure.resetEle());
      controller2.leftBumper().onTrue(superstructure.resetWrist());


      controller2.getUpButton().onTrue(new SuperToState(superstructure, SuperPreset.START.getState()));
      controller2.getRightButton().onTrue(new SuperToState(superstructure, SuperPreset.SOURCE.getState()));
      controller2.getLeftButton().onTrue(new SuperToState(superstructure, SuperPreset.L3_CORAL.getState()));
      controller2.getDownButton().onTrue(new SuperToState(superstructure, SuperPreset.L2_ALGAE_REVERSE.getState()));
  }

  private void configureDashboard() {
    

    // Set up auto routines

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    autoChooser.addOption("Mid to 2 corals gui", AutoTests.twoCoral());
    autoChooser.addOption("left source coral", AutoFactory.leftL4CoralGen(drive, superstructure, intake));
    autoChooser.addOption("right source coral", AutoFactory.rightL4CoralGen(drive, superstructure, intake));

    //drive to pose cmmd test
    autoChooser.addOption("2 corals drive", AutoTests.drive2Corals(drive));
    autoChooser.addOption("pathgen", AutoTests.AG2Coral(drive));
    
    GenAutoChooser.getInstance().init();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();

    //return GenAutoChooser.getInstance().getAutonomousCommand(drive, superstructure, intake);
  }

  public void resetSimulation(){
    if (Constants.robot.currMode != Constants.Mode.SIM) return;
    superstructure.setTarget(SuperState.SuperPreset.START.getState());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.robot.currMode != Constants.Mode.SIM) return;
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()});

    Logger.recordOutput(
            "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
            "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}