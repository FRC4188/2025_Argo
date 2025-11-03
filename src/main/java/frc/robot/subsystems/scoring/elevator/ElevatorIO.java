
package frc.robot.subsystems.scoring.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;
        public double positionMeters = 0.0;
        public double velocityMeters = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setOpenLoop(double output) {}

    public default void setPosition(double meters) {}
    
    public default void updatePIDGains(double p, double i, double d) {}
}