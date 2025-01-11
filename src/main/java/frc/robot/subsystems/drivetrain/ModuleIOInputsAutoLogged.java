package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

public class ModuleIOInputsAutoLogged extends ModuleIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("Drive Connection", driveConnected);
        table.put("Drive Position (rad)", drivePositionRad);
        table.put("Drive Velocity (rad/s)", driveVelocityRadPerSec);
        table.put("Drive Applied Volts", driveAppliedVolts);
        table.put("Drive Current (A)", driveCurrentAmps);

        table.put("Turn Connection", turnConnected);
        table.put("Turn Encoder Connection", turnEncoderConnected);
        table.put("Turn Absolute Position", turnAbsolutePosition.getRadians());
        table.put("Turn Position", turnPosition.getRadians());
        table.put("Turn Velocity (rad/s)", turnVelocityRadPerSec);
        table.put("Turn Applied Volts", turnAppliedVolts);
        table.put("Turn Current (A)", turnCurrentAmps);

        table.put("Odometry Timestamps", odometryTimestamps);
        table.put("Odometry Drive Positions (rad)", odometryDrivePositionsRad);
        table.put("Odometry Turn Positions", odometryTurnPositions);

    }

    @Override
    public void fromLog(LogTable table) {
        driveConnected = table.get("Drive Connection", driveConnected);
        drivePositionRad = table.get("Drive Position (rad)", drivePositionRad);
        driveVelocityRadPerSec = table.get("Drive Velocity (rad/s)", driveVelocityRadPerSec);
        driveAppliedVolts = table.get("Drive Applied Volts", driveAppliedVolts);
        driveCurrentAmps = table.get("Drive Current (A)", driveCurrentAmps);

        turnConnected = table.get("Turn Connection", turnConnected);
        turnEncoderConnected = table.get("Turn Encoder Connection", turnEncoderConnected);
        turnAbsolutePosition = new Rotation2d(table.get("Turn Absolute Position", turnAbsolutePosition.getRadians()));
        turnPosition = new Rotation2d(table.get("Turn Position", turnPosition.getRadians()));
        turnVelocityRadPerSec = table.get("Turn Velocity (rad/s)", turnVelocityRadPerSec);
        turnAppliedVolts = table.get("Turn Applied Volts", turnAppliedVolts);
        turnCurrentAmps = table.get("Turn Current (A)", turnCurrentAmps);

        odometryTimestamps = table.get("Odometry Timestamps", odometryTimestamps);
        odometryDrivePositionsRad = table.get("Odometry Drive Positions (rad)", odometryDrivePositionsRad);
        odometryTurnPositions = table.get("Odometry Turn Positions", odometryTurnPositions);
        
    }
    
}
