package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.pathgen.PathGen;
import frc.robot.CSPLib.pathgen.fieldobjects.*;
import frc.robot.CSPLib.pidtuning.PIDTuning;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.intake.IntakeCommands;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive;
  private final Vision vis;
  private final SuperStructure superstruct;
  private final Intake intake;
  private PIDTuning tuner = null;

  private final CSP_Controller pilot = new CSP_Controller(0);
  private final CSP_Controller copilot = new CSP_Controller(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vis = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        superstruct = new SuperStructure(new ElevatorIOSim(), new WristIOSim());
        intake = new Intake(new IntakeIOSim());
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vis = new Vision(drive::addVisionMeasurement, new VisionIO() {});

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

    PathGen.getInstance()
        .configure(
            0.1,
            Constants.robot.B_CROSSLENGTH,
            new PolygonFO(
                true,
                FieldConstant.Reef.Base.left_brg_corner,
                FieldConstant.Reef.Base.right_brg_corner,
                FieldConstant.Reef.Base.right_field_corner,
                FieldConstant.Reef.Base.right_src_corner,
                FieldConstant.Reef.Base.left_src_corner,
                FieldConstant.Reef.Base.left_field_corner),
            new PolygonFO(
                true,
                FieldConstant.Field.all_wall_left_corner,
                FieldConstant.Field.alliance_left_corner,
                FieldConstant.Field.alliance_right_corner,
                FieldConstant.Field.all_wall_right_corner,
                FieldConstant.Field.mid_right_wall,
                FieldConstant.Field.mid_left_wall));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");//, AutoBuilder.buildAutoChooser());

    configureDashboard();
    configureButtonBindings();
  }

  public void autoInit() {
    superstruct.setState(SuperState.SuperPreset.START.getState());
    FieldConstant.Reef.AlgaeSource.reloadAsources();
    FieldConstant.Reef.AlgaeSource.reloadAsources();
  }

  public void teleInit() {
    superstruct.setState(SuperState.SuperPreset.START.getState());
    intake.runVolts(0.0);
    FieldConstant.Reef.AlgaeSource.reloadAsources();
    FieldConstant.Reef.AlgaeSource.reloadAsources();
  }

  public void telePeriodic() {
    if (tuner != null) tuner.updateLoop();
  }

  private void configureButtonBindings() {
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
                    () -> drive.getPose().getTranslation().getAngle())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));

    pilot
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d()), // drive.getPose().getTranslation(), new Rotation2d())),
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

    intakeInput
        .whileTrue(
            Commands.run(
                () ->
                    intake.runVolts(
                        12 * (pilot.getLeftT(Scale.LINEAR) - pilot.getRightT(Scale.LINEAR))),
                intake))
        .onFalse(Commands.runOnce(intake::stop, intake));

    pilot
        .getRightTButton()
        .and(pilot.leftBumper())
        .whileTrue(IntakeCommands.unstickIntake(intake))
        .onFalse(Commands.runOnce(intake::stop, intake));

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

    copilot
        .rightBumper()
        .onTrue((new ScoreNet(superstruct, intake)).handleInterrupt(() -> intake.stop()));

    copilot
        .x()
        .and(copilot.leftBumper())
        .onTrue(Commands.runOnce(superstruct::resetElevator, superstruct));
  }

  private void configureDashboard() {
    // Sequences for Tuning
    // Sequences for Tuning
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

    // Actual Autos
    autoChooser.addOption("Push", Commands.sequence(AutoScore.pushLeave(drive)));
    // Actual Autos
    autoChooser.addOption("Push", Commands.sequence(AutoScore.pushLeave(drive)));

    autoChooser.addOption(
        "Coral and 2 Processor",
        Commands.sequence(
            new AutoScore.coralScore(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake)));

    autoChooser.addOption(
        "2 Processor",
        Commands.sequence(
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake),
            new AutoScore.algaeProcess(drive, superstruct, intake)));

    autoChooser.addOption(
        "Coral and 1.5 Net",
        Commands.sequence(
            new AutoScore.coralScore(drive, superstruct, intake),
            new AutoScore.algaeNet(drive, superstruct, intake),
            new AutoScore.algaeSource(drive, superstruct, intake)));

    autoChooser.addOption(
        "Coral", Commands.sequence(new AutoScore.coralScore(drive, superstruct, intake)));

    autoChooser.addOption(
        "Ring Around The Rosie",
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
        "Ring Around One Rosie",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.left_src_src),
            new DriveTo(drive, FieldConstant.start_right),
            new DriveTo(drive, FieldConstant.Source.left_src_mid)));

    autoChooser.addOption(
        "Test Pathing",
        Commands.sequence(
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.mid_brg_src),
            new DriveTo(drive, FieldConstant.Processor.processor_goal),
            new DriveTo(drive, FieldConstant.Reef.AlgaeSource.right_brg_src),
            new DriveTo(drive, FieldConstant.Processor.processor_goal)));

    autoChooser.addOption("Path Planner Test", getAutonomousCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Auto");
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Uh oh" + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }

    // undo the below comment later ig
    // return autoChooser.get();
  }
}
