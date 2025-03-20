
package frc.robot.subsystems.scoring.elevator;

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
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runPos(double pos){}

    public default double getHeight(){return 0;}
}
