
package frc.robot.subsystems.scoring.wrist;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.scoring.intake.IntakeIO.IntakeIOInputs;


public interface WristIO {//J.C
    @AutoLog
    public static class WristIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;
        public double velRadsPerSec = 0.0;
    }

    default void updateInputs(WristIOInputs inputs) {}
    default void runVolts(double volts) {}

}
