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

// import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveTo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.lift.*;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.wrist.*;
import frc.robot.util.FieldConstant;
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
  private Vision vis;
  private final Elevator elevator;
  private final Wrist wrist;
  private final Intake intake;

  // pilot
  private final CSP_Controller pilot = new CSP_Controller(0);
  private final CSP_Controller copilot = new CSP_Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vis =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhoton(VisConstants.frontPho, VisConstants.robotToCamera0));

        elevator = new Elevator(new ElevatorIOReal());
        intake = new Intake(new IntakeIOReal());
        wrist = new Wrist(new WristIOReal());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        elevator = new Elevator(new ElevatorIOSim());
        intake = new Intake(new IntakeIOSim());
        wrist = new Wrist(new WristIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = new Elevator(new ElevatorIO() {});
        intake = new Intake(new IntakeIO() {});
        wrist = new Wrist(new WristIO() {});

        break;
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices"); // , AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureDashboard();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link Xboxpilot}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // rahhhhhhhhhhhhhhhhh
    Trigger driveInput =
        new Trigger(
            () ->
                (pilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0
                    || pilot.getCorrectedRight(Scale.LINEAR).getX() != 0.0));

    Trigger eleInput = new Trigger(() -> (copilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0));

    Trigger wristInput =
        new Trigger(() -> (copilot.getCorrectedRight(Scale.LINEAR).getNorm() != 0.0));

    drive.setDefaultCommand(Commands.runOnce(drive::stopWithX, drive));
    elevator.setDefaultCommand(Commands.run(() -> elevator.setHeight(0.0), elevator));
    wrist.setDefaultCommand(Commands.run(() -> wrist.setPosition(0.0), wrist));
    intake.setDefaultCommand(
        Commands.run(
            () ->
                intake.runVolts(
                    12 * (copilot.getLeftT(Scale.LINEAR) - copilot.getRightT(Scale.LINEAR))),
            intake));

    copilot.a().onTrue(Commands.runOnce(elevator::setZero, elevator));

    eleInput.whileTrue(
        Commands.run(() -> elevator.runVolts(12 * copilot.getLeftY(Scale.LINEAR)), elevator));

    wristInput.whileTrue(
        Commands.run(() -> wrist.runVolts(3 * copilot.getRightY(Scale.LINEAR)), wrist));

    driveInput.whileTrue(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                    * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
            () ->
                -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                    * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
            () ->
                -pilot.getCorrectedRight(Scale.SQUARED).getX()
                    * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0)));

    pilot
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                            * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                            * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                    () ->
                        FieldConstant.Reef.center
                            .minus(drive.getPose().getTranslation())
                            .getAngle())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    pilot.x().onTrue(Commands.runOnce(() -> drive.acceptVision(true), drive));
    pilot.y().onTrue(Commands.runOnce(() -> drive.acceptVision(false), drive));

    switch (Constants.pid_mode) {
      case DRIVE_MOD:
        pilot.b().onTrue(Commands.runOnce(drive::updateDrivePID, drive));
        break;
      case TURN_MOD:
        pilot.b().onTrue(Commands.runOnce(drive::updateTurnPID, drive));
        break;
      case ANGLE_PROF:
        pilot.b().onTrue(Commands.runOnce(DriveCommands::updateAnglePID, drive));
        break;

      case ELEVATOR:
        copilot.b().onTrue(Commands.runOnce(elevator::updatePID, drive));
        break;
      case WRIST:
        copilot.b().onTrue(Commands.runOnce(wrist::updatePID, drive));
        break;
      case NONE:
      default:
    }

    // Reset gyro to 0° when B button is pressed
    pilot
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private void configureDashboard() {

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

    autoChooser.addOption(
        "Trajectory Test",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.alliance_src),
            new DriveTo(drive, FieldConstant.Processor.processor_goal)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
