package frc.robot.subsystems.scoring.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;
    // maybe work
    
    // need to be calibrated
    double armZero = 0;
    PIDController armPID = new PIDController(0.0, 0.0, 0.0);
    
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 3;
    
    public static Arm getInstance(ArmIO io) {
        if (instance == null) {
            instance = new Arm(io);
        }
        return instance;
    }

    public enum ArmPreset{
        MAX,
        MIN
        }
    
    public Arm(ArmIO io) {
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
    
    public Command setAngle(double angle) {
     return Commands.run(()->{
        // why is it angle two? becuase thats what makes it stop yelling
        double angle2 = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
        double currentAngle = inputs.positionRads-armZero;
        double output = armPID.calculate(currentAngle, angle2);
        io.runVolts(output);});
    }
    
    public Command setVolts(double percent){
        return Commands.run(()->{
            io.runVolts(percent);
        });
    }

   

    
    // Commands need to be reviewed may have implemented them incorrectly
    public Command stopArm(Arm arm) {
        return Commands.run(()->{ io.stop();});
    }
    // The following commands will be used for going to 

    public double getAngle() {
        return io.getAngle();
    }
    
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getAngle() - targetAngle) < armPID.getPositionTolerance();
    }
    
    public void resetAngleZero() {
        armZero = inputs.positionRads;
    }
}