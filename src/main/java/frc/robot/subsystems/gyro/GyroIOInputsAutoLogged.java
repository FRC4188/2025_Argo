package frc.robot.subsystems.gyro;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.gyro.GyroIO.GyroIOInputs;

public class GyroIOInputsAutoLogged extends GyroIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("connected", connected);
        table.put("yawPosition", yawPosition.getDegrees());
        table.put("yawVelocityRadPerSec", yawVelocityRadPerSec);
        table.put("odometryYawTimestamps", odometryYawTimestamps);
        table.put("odometryYawPositions", odometryYawPositions);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("connected", connected);
        yawPosition = Rotation2d.fromDegrees(table.get("yawPosition", yawPosition.getDegrees()));
        yawVelocityRadPerSec = table.get("yawVelocityRadPerSec", yawVelocityRadPerSec);
        odometryYawTimestamps = table.get("odometryYawTimestamps", odometryYawTimestamps);
        odometryYawPositions = table.get("odometryYawPositions", odometryYawPositions);
    }   
    
}
