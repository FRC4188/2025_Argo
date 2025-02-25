
package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drive;

/**
 * Drives to a specified pose.
 */
public class DriveToPose extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            Constants.robot.DRIVE_PID.kP, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.robot.TURN_PID.kP, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private Drive driveSubsystem;
    private Supplier<Pose2d> poseSupplier;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double thetaVelocity = 0;
    private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

    public DriveToPose(Drive driveSubsystem, Supplier<Pose2d> poseSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.poseSupplier = poseSupplier;
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveSubsystem.getState().Pose;
        driveController.reset(
                currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(driveSubsystem.getState().Speeds.vyMetersPerSecond,
                                driveSubsystem.getState().Speeds.vxMetersPerSecond)
                                .rotateBy(
                                        poseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(driveSubsystem.getState().Pose.getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(),
                driveSubsystem.getState().Speeds.omegaRadiansPerSecond);
        lastSetpointTranslation = driveSubsystem.getState().Pose.getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getState().Pose;
        Pose2d targetPose = poseSupplier.get();

        Logger.recordOutput("Drive/DriveToPose/currentPose", currentPose);
        Logger.recordOutput("Drive/DriveToPose/targetPose", targetPose);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        new Transform2d(
                                new Translation2d(driveController.getSetpoint().position, 0.0), new Rotation2d()))
                .getTranslation();

        // Calculate theta speed
        thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity = new Pose2d(new Translation2d(), currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
                .getTranslation();
                
        driveSubsystem.applyRequest(() -> new SwerveRequest.ApplyFieldSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.applyRequest(()-> new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public boolean isFinished() {
        return poseSupplier.get().equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }
}
