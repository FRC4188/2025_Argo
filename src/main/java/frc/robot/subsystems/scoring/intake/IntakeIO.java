package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.AutoLog;

// i added stuff in here - anish
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
    }
   


    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default boolean isIn(){
        return true;
    }

    public default boolean isStalled() {return false;}

    public default void stop(){}
}