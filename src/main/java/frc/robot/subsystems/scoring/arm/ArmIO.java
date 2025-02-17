package frc.robot.subsystems.scoring.arm;

import static frc.robot.subsystems.scoring.superstructure.SuperstructureConfig.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public boolean connected = true;
        public double appliedVolts = 0.0;
        public double tempC = 0.0;
        public double positionRads = 0.0;
        public double desiredPositionRads = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void runVolts(double volts) {}
    public default double getAngle(){ return  0;}
    public default void stop() {}
}