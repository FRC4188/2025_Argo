package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

// i added stuff in here - anish
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;
        public double velRadsPerSec = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void invertMotor(boolean isInverted) {}

    public default boolean isSafetyOn(boolean isSafe) {return isSafe;}

    public default void stop(){}
}
