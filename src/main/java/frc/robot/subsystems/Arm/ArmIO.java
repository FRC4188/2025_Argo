package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double positionRads = 0.0;
        public double velocityRadPerSec = 0.0;
        public double desiredPositionRads = 0.0;
        public double desiredVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void runVolts(double volts) {}
    public default void stop() {}
}
