package frc.robot.subsystems.scoring.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeWristIO {
    @AutoLog
    public static class IntakeWristIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;
        public double velRadsPerSec = 0.0;
    }

    public default void updateInputs(IntakeWristIOInputs inputs) {

    }

}
