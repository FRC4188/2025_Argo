package frc.robot.commands.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.DriveTo;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;
import java.util.Optional;

public class AutoScore extends Command {
  protected Drive drive;
  protected SuperStructure superstruct;
  protected Intake intake;

  protected Pose2d goal;
  protected SuperPreset preset;
  protected Command scoring;
  protected boolean presetGoal = false;

  public void factory() {}

  public final void initialize() {
    factory();
    addRequirements(drive, superstruct, intake);
    scoring.initialize();
  }

  public final void execute() {
    scoring.execute();
  }

  public final boolean isFinished() {
    return scoring.isFinished();
  }

  public final void end(boolean interrupted) {
    scoring.end(interrupted);
  }

  public static class algaeSource extends AutoScore {

    public algaeSource(Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
    }

    public algaeSource(Pose2d pose, Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
      goal = pose;
      presetGoal = true;
    }

    @Override
    public void factory() {

      if (!presetGoal) {
        goal =
            AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);
      }

      FieldConstant.Reef.AlgaeSource.asources.remove(goal);
      int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);

      if (height == 3) {
        preset = SuperPreset.L3_ALGAE;
      } else {
        preset = SuperPreset.L2_ALGAE;
      }

      scoring =
          Commands.sequence(
              Commands.parallel(
                  new DriveTo(drive, goal),
                  Commands.sequence(
                      new WaitUntilCommand(
                          () ->
                              AllianceFlip.flipDS(drive.getPose())
                                      .getTranslation()
                                      .getDistance(goal.getTranslation())
                                  < 2),
                      SuperCommands.superToState(superstruct, preset.getState(), Rotation2d.kZero),
                      SuperCommands.superToState(superstruct, Rotation2d.fromRadians(0.6)))),
              Commands.run(() -> intake.runVolts(7))
                  .until(() -> intake.isStalled())
                  .withTimeout(0.5),
              Commands.runOnce(intake::stop));
    }
  }

  public static class algaeProcess extends AutoScore {
    public algaeProcess(Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
      goal = FieldConstant.Processor.processor_goal;
    }

    @Override
    public void factory() {
      preset = SuperPreset.PROCESSOR;

      scoring =
          Commands.sequence(
              SuperCommands.superToState(superstruct, Rotation2d.fromRadians(0.3)),
              Commands.parallel(
                  new DriveTo(drive, goal),
                  Commands.sequence(
                      new WaitCommand(0.5),
                      SuperCommands.superToState(superstruct, SuperPreset.PROCESSOR.getState()),
                      Commands.runOnce(() -> superstruct.setWrist(Rotation2d.kZero)))),
              Commands.run(() -> intake.runVolts(-10)).withTimeout(1),
              Commands.runOnce(intake::stop));
    }
  }

  public static class algaeNet extends AutoScore {
    public algaeNet(Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
    }

    public algaeNet(Pose2d pose, Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
      goal = pose;
      presetGoal = true;
    }

    public algaeNet(
        Optional<Pose2d> pose, Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
      if (pose.isPresent()) goal = pose.get();
      presetGoal = true;
    }

    @Override
    public void factory() {
      if (!presetGoal) {
        goal = AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Net.nscores);
      }
      FieldConstant.Net.nscores.remove(goal);

      scoring =
          Commands.sequence(
              new DriveTo(drive, goal),
              Commands.runOnce(drive::stopWithX, drive),
              new ScoreNet(superstruct, intake));
    }
  }

  public static Command pushLeave(Drive drive) {
    return drive.sysIdDynamic(Direction.kForward).withTimeout(15);
  }

  public static class coralScore extends AutoScore {

    public coralScore(Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
    }

    public coralScore(Pose2d pose, Drive drive, SuperStructure superstructure, Intake intake) {
      this.drive = drive;
      this.superstruct = superstructure;
      this.intake = intake;
      goal = pose;
      presetGoal = true;
    }

    @Override
    public void factory() {
      if (!presetGoal) {
        goal =
            AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);
      }

      FieldConstant.Reef.AlgaeSource.asources.remove(goal);
      int height = FieldConstant.Reef.AlgaeSource.algaeHeight(goal);

      if (height == 3) {
        preset = SuperPreset.L3_ALGAE;
      } else {
        preset = SuperPreset.L2_ALGAE;
      }

      scoring =
          Commands.sequence(
              Commands.race(
                  Commands.parallel(
                      Commands.sequence(
                          new DriveTo(drive, goal), Commands.runOnce(drive::stopWithX, drive)),
                      Commands.sequence(
                          new WaitUntilCommand(
                              () ->
                                  AllianceFlip.flipDS(drive.getPose())
                                          .getTranslation()
                                          .getDistance(goal.getTranslation())
                                      < 0.5),
                          SuperCommands.superToState(
                              superstruct, SuperPreset.L1_CORAL.getState(), Rotation2d.kZero)))),
              SuperCommands.superToState(superstruct, preset.getState(), Rotation2d.kZero),
              SuperCommands.superToState(superstruct, Rotation2d.fromRadians(0.6)),
              Commands.run(() -> intake.runVolts(7))
                  .until(() -> intake.isStalled())
                  .withTimeout(1.5),
              Commands.run(() -> intake.runVolts(7)).withTimeout(0.5),
              Commands.runOnce(intake::stop));
    }

    public static class coralOnly extends AutoScore {

      public coralOnly(Drive drive, SuperStructure superstructure, Intake intake) {
        this.drive = drive;
        this.superstruct = superstructure;
        this.intake = intake;
      }

      public coralOnly(Pose2d pose, Drive drive, SuperStructure superstructure, Intake intake) {
        this.drive = drive;
        this.superstruct = superstructure;
        this.intake = intake;
        goal = pose;
        presetGoal = true;
      }

      @Override
      public void factory() {
        if (!presetGoal) {
          goal =
              AllianceFlip.flipDS(drive.getPose()).nearest(FieldConstant.Reef.AlgaeSource.asources);
        }

        scoring =
            Commands.sequence(
                Commands.race(
                    Commands.parallel(
                        Commands.sequence(
                            new DriveTo(drive, goal), Commands.runOnce(drive::stopWithX, drive)),
                        Commands.sequence(
                            new WaitUntilCommand(
                                () ->
                                    AllianceFlip.flipDS(drive.getPose())
                                            .getTranslation()
                                            .getDistance(goal.getTranslation())
                                        < 2),
                            SuperCommands.superToState(
                                superstruct, SuperPreset.L1_CORAL.getState(), Rotation2d.kZero)))));
      }
    }
  }
}
