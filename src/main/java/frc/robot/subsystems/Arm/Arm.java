package frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private static Arm instance = null;
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;
    // maybe work
    
    // need to be calibrated
    double armZero = 0;
    PIDController armPID = new PIDController(0.1, 0.0, 0.0);
    
    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 3;
    
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
    
    public Command setAngle(Arm arm, double angle) {
     return Commands.run(()->{
        // why is it angle two? becuase thats what makes it stop yelling
        double angle2 = Math.min(Math.max(angle, MIN_ANGLE), MAX_ANGLE);
        double currentAngle = inputs.positionRads-armZero;
        double output = armPID.calculate(currentAngle, angle2);
        io.runVolts(output);}, arm);
    }


    
    // Commands need to be reviewed may have implemented them incorrectly
    public Command stopArm(Arm arm) {
        return Commands.run(()->{ io.stop();}, arm);
    }
    // The following commands will be used for going to 
    public Command customAngle1 (Arm arm){
        return Commands.run(()->{
         setAngle(arm, MAX_ANGLE);
        }, arm);
    }

    public Command customAngle2(Arm arm){
        return Commands.run(()->{
            setAngle(arm, MIN_ANGLE);
        }, arm);
    }
    
    public  double getAngle() {
        return inputs.positionRads - armZero;
    }
    
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getAngle() - targetAngle) < armPID.getPositionTolerance();
    }
    
    public void resetAngleZero() {
        armZero = inputs.positionRads;
    }
}