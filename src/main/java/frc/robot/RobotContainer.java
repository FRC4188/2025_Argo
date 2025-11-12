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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.pidtuning.PIDTuning;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.superstructure.AutoScore;
import frc.robot.commands.superstructure.ScoreNet;
import frc.robot.commands.superstructure.SuperCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.superstructure.SuperState;
import frc.robot.subsystems.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.wrist.WristIO;
import frc.robot.subsystems.superstructure.wrist.WristIOReal;
import frc.robot.subsystems.superstructure.wrist.WristIOSim;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;
import java.util.Arrays;
import java.util.LinkedList;
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
  private final SuperStructure superstruct;
  private final Intake intake;
  private PIDTuning tuner = null;

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
                new VisionIOPhoton(VisConstants.frontPho, VisConstants.robotToCamera0),
                new VisionIOPhoton(VisConstants.backPho, VisConstants.robotToCamera2));

        superstruct = new SuperStructure(new ElevatorIOReal(), new WristIOReal());
        intake = new Intake(new IntakeIOReal());

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

        superstruct = new SuperStructure(new ElevatorIOSim(), new WristIOSim());
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
                new ModuleIO() {});

        superstruct = new SuperStructure(new ElevatorIO() {}, new WristIO() {});

        intake = new Intake(new IntakeIO() {});

        break;
    }

    switch (Constants.pid_mode) {
      case DRIVE_MOD:
        tuner = new PIDTuning("Drive Modules", () -> 0, (set) -> {}, drive::updateDrivePID);
        break;
      case TURN_MOD:
        tuner = new PIDTuning("Turn Modules", () -> 0, (set) -> {}, drive::updateTurnPID);
        break;
      case ANGLE_PROF:
        tuner =
            new PIDTuning(
                "Angle Controller",
                () -> drive.getPose().getRotation().getRadians(),
                (set) -> {},
                DriveCommands::updateAnglePID);

        break;
      case ELEVATOR:
        tuner =
            new PIDTuning(
                "Elevator",
                () -> superstruct.getState().getEleHeight(),
                (set) -> {
                  superstruct.setElevator(set);
                },
                superstruct::updateElePID);
        break;
      case WRIST:
        tuner =
            new PIDTuning(
                "Wrist",
                () -> superstruct.getState().getWristAngle().getRadians(),
                (set) -> {
                  superstruct.setWrist(Rotation2d.fromRadians(set));
                },
                superstruct::updateWristPID);
        break;
      case INTAKE:
        tuner =
            new PIDTuning(
                "Intake",
                () -> intake.getRPM(),
                (set) -> {
                  intake.setVelocityRPM(set);
                },
                intake::updatePID);
        break;
      case NONE:
      default:
    }

    // Set up auto routines
    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices"); // , AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureDashboard();
    configureButtonBindings();
  }

  public void autoInit() {
    superstruct.setState(SuperState.SuperPreset.START.getState());
    FieldConstant.Reef.AlgaeSource.asources =
        new LinkedList<Pose2d>(
            Arrays.asList(
                FieldConstant.Reef.AlgaeSource.alliance_src,
                FieldConstant.Reef.AlgaeSource.left_brg_src,
                FieldConstant.Reef.AlgaeSource.left_src_src,
                FieldConstant.Reef.AlgaeSource.right_brg_src,
                FieldConstant.Reef.AlgaeSource.right_src_src,
                FieldConstant.Reef.AlgaeSource.mid_brg_src));
  }

  public void teleInit() {
    superstruct.setState(SuperState.SuperPreset.START.getState());
    intake.runVolts(0.0);
    FieldConstant.Reef.AlgaeSource.asources =
        new LinkedList<Pose2d>(
            Arrays.asList(
                FieldConstant.Reef.AlgaeSource.alliance_src,
                FieldConstant.Reef.AlgaeSource.left_brg_src,
                FieldConstant.Reef.AlgaeSource.left_src_src,
                FieldConstant.Reef.AlgaeSource.right_brg_src,
                FieldConstant.Reef.AlgaeSource.right_src_src,
                FieldConstant.Reef.AlgaeSource.mid_brg_src));
  }

  public void telePeriodic() {
    if (tuner != null) tuner.updateLoop();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    Trigger driveInput =
        new Trigger(
            () ->
                (pilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0
                    || pilot.getCorrectedRight(Scale.LINEAR).getX() != 0.0));

    driveInput
        .whileTrue(
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
                        * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0)))
        .onFalse(Commands.runOnce(drive::stop, drive));

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
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));

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

    pilot
        .x()
        .and(pilot.leftBumper())
        .onTrue(Commands.runOnce(() -> drive.acceptVision(true), drive));
    pilot
        .y()
        .and(pilot.leftBumper())
        .onTrue(Commands.runOnce(() -> drive.acceptVision(false), drive));

    pilot
        .b()
        .whileTrue(
            new DriveToPose(
                    drive,
                    () ->
                        AllianceFlip.flipDS(drive.getPose())
                            .nearest(FieldConstant.Reef.AlgaeSource.asources))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));

    Trigger intakeInput =
        new Trigger(
            () -> (pilot.getLeftT(Scale.LINEAR) != 0.0 || pilot.getRightT(Scale.LINEAR) != 0.0));

    // temp command
    intakeInput
        .whileTrue(
            Commands.run(
                () ->
                    intake.runVolts(
                        12 * (pilot.getLeftT(Scale.LINEAR) - pilot.getRightT(Scale.LINEAR))),
                intake))
        .onFalse(Commands.runOnce(intake::stop, intake));

    // intakeInput.whileTrue(IntakeCommands.driveIntake(intake, () ->
    // (copilot.getLeftT(Scale.LINEAR) - copilot.getRightT(Scale.LINEAR))), intake);

    Trigger superInput =
        new Trigger(
            () ->
                (copilot.getCorrectedLeft(Scale.LINEAR).getY() != 0.0
                    || copilot.getCorrectedRight(Scale.LINEAR).getY() != 0.0));

    superInput.whileTrue(
        SuperCommands.superDrive(
            superstruct,
            () ->
                -copilot.getCorrectedLeft(Scale.SQUARED).getY()
                    * (copilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
            () ->
                -copilot.getCorrectedRight(Scale.SQUARED).getY()
                    * (copilot.rightBumper().getAsBoolean() ? 0.5 : 1.0)));

    copilot
        .getUpButton()
        .onTrue(
            SuperCommands.superToState(
                superstruct, SuperPreset.PROCESSOR.getState(), Rotation2d.fromRadians(0.5)));

    copilot
        .getLeftButton()
        .onTrue(
            SuperCommands.superToState(
                superstruct, SuperPreset.L2_ALGAE.getState(), Rotation2d.fromRadians(0.0)));

    copilot
        .getRightButton()
        .onTrue(
            SuperCommands.superToState(
                superstruct, SuperPreset.L3_ALGAE.getState(), Rotation2d.fromRadians(0.0)));

    copilot
        .getDownButton()
        .onTrue(
            SuperCommands.superToState(
                superstruct, SuperPreset.ALGAE_GROUND.getState().getWristAngle()));

    copilot
        .start()
        .onTrue(
            SuperCommands.superToState(
                superstruct, SuperPreset.START.getState(), Rotation2d.fromRadians(0.0)));

    copilot.rightBumper().onTrue(new ScoreNet(superstruct, intake));

    copilot
        .x()
        .and(copilot.leftBumper())
        .onTrue(Commands.runOnce(superstruct::resetElevator, superstruct));
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

    autoChooser.addOption("leave ", Commands.sequence(AutoScore.pushLeave(drive)));

    autoChooser.addOption(
        "coral n process",
        Commands.sequence(
            new AutoScore.coralScore(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake)));

    autoChooser.addOption(
        "process",
        Commands.sequence(
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake)));

    autoChooser.addOption(
        "net",
        Commands.sequence(
            new AutoScore.coralScore(drive, superstruct, intake),
            new AutoScore.algaeNet(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake)));

    autoChooser.addOption(
        "coral", Commands.sequence(new AutoScore.coralScore(drive, superstruct, intake)));

    autoChooser.addOption(
        "ring around the rosie",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.alliance_src),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.mid_brg_src),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_src_src),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.right_brg_src),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_brg_src),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.right_src_src),
            new DriveTo(drive, FieldConstant.Source.left_src_mid),
            new DriveTo(drive, FieldConstant.Processor.processor_goal),
            new DriveTo(drive, FieldConstant.Source.right_src_mid),
            new DriveTo(drive, FieldConstant.start_mid),
            new DriveTo(drive, FieldConstant.start_left),
            new DriveTo(drive, FieldConstant.start_right)));

    autoChooser.addOption(
        "ring around one rosie",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_src_src),
            new DriveTo(drive, FieldConstant.start_right),
            new DriveTo(drive, FieldConstant.Source.left_src_mid)));

    autoChooser.addOption(
        "Test pathing",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.mid_brg_src),
            new DriveTo(drive, FieldConstant.Processor.processor_goal),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.right_brg_src),
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
