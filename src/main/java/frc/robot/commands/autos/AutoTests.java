package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTo;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.scoring.AutoScore;
import frc.robot.commands.scoring.SuperToState;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstant;

public final class AutoTests {
    private final static PathConstraints  constraints =  new PathConstraints(
            TunerConstants.kSpeedAt12Volts,
            MetersPerSecondPerSecond.of(3.6),
            DegreesPerSecond.of(TunerConstants.kSpeedAt12Volts.magnitude()),
            DegreesPerSecondPerSecond.of(862),
            Volts.of(12),
            false);

    public static Command init(Pose2d pose, Drive drive, Superstructure superstructure) {
        return Commands.runOnce(()->drive.setPose(AllianceFlip.flipDS(pose))).alongWith(
            Commands.runOnce(() -> superstructure.resetEle())
        );
    }

}
