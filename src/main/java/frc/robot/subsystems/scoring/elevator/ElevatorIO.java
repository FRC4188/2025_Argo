
package frc.robot.subsystems.scoring.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double posRads = 0.0;

        public double followerTempC = 0.0;
        public double followerAppliedVolts = 0.0;
    }

    public default ProfiledPIDController getPID() {return new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));}
    public default ElevatorFeedforward getFF() {return new ElevatorFeedforward(0, 0, 0);}

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default double getHeight(){return 0;}
}