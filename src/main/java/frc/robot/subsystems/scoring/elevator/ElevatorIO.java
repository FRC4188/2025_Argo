
package frc.robot.subsystems.scoring.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = true;
        public double velRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;

        public double followerTempC = 0.0;
        public double followerAppliedVolts = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runPosition(double height, double ff){}
}
