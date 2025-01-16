package frc.robot.subsystems.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation sim;

    public GyroIOSim(GyroSimulation sim) {
        this.sim = sim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = true;
        inputs.yawPosition = sim.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
            sim.getMeasuredAngularVelocity().in(RadiansPerSecond)
        );
        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = sim.getCachedGyroReadings();
    }
}
