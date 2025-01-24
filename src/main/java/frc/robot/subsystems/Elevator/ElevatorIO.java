package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = true;
        public double appliedVoltsLeft = 0.0;
        public double tempCLeft = 0.0;
        public double posRadsLeft = 0.0;
        public double velRadsPerSecLeft = 0.0;
        public double appliedVoltsRight = 0.0;
        public double tempCRight = 0.0;
        public double posRadsRight = 0.0;
        public double velRadsPerSecRight = 0.0;
        public double desiredPos = 0.0;
        public double desiredVel = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void stop(){}
}