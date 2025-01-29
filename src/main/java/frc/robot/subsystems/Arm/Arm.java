package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;
    // maybe work
    
  
    double armZero = 0;
    PIDController armPID = new PIDController(0.1, 0.0, 0.0);
    
    private static final double MIN_ANGLE = 0.0;
    private static final double MAX_ANGLE = 270.0;
    
    public static Arm getInstance(ArmIO io) {
        if (instance == null) {
            instance = new Arm(io);
        }
        return instance;
    }
    
    private Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIOInputsAutoLogged();
        armPID.enableContinuousInput(0, 360);
        armPID.setTolerance(2.0);
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }
    
    public void setAngle(double angle) {
        angle = Math.min(Math.max(angle, MIN_ANGLE), MAX_ANGLE);
        double currentAngle = getAngle();
        double output = armPID.calculate(currentAngle, angle);
        io.runVolts(output);
    }
    
    public void stopArm() {
        io.stop();
    }
    
    public double getAngle() {
        return inputs.positionRads - armZero;
    }
    
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getAngle() - targetAngle) < armPID.getPositionTolerance();
    }
    
    public void resetAngleZero() {
        armZero = inputs.positionRads;
    }
}