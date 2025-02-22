
package frc.robot.subsystems.scoring.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;


public interface WristIO {//J.C
    @AutoLog
    public static class WristIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;
        public double desiredPositionRads = 0.0;
    }
    default void updateInputs(WristIOInputs inputs) {}
    default void runVolts(double volts) {}
    default void setPower(double power) {}
    default double getAngle() {return 0;}

}
