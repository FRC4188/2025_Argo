
package frc.robot.subsystems.scoring.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = true;
        
        public double posMeters = 0.0;

        public double appliedVolts = 0.0;
        public double tempC = 0.0;

        public double followerTempC = 0.0;
        public double followerAppliedVolts = 0.0;

        public double getVelocityMetersPerSec;

        public double velocityRotsPerSec;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}
    
    public default double getHeight(){return 0;}

    public default boolean isStalled() {return false;}

    public default void setLeaderOpenLoop(double output) {}

    public default void setMotorPosition(DoubleSupplier target) {}

    public default void setHeight(DoubleSupplier target) {}

    public default void setElevatorVelocity(double velocityRadPerSec) {}
}