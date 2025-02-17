package frc.robot.subsystems.scoring.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;
    // maybe work
    
    // need to be calibrated
    double armZero = 0;
    // PIDController armPID = new PIDController(0.0, 0.0, 0.0);
    double tolerance = 0.05;
    double target = 0;
    

    public enum ArmPreset{
        MAX,
        MIN
        }
    
    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIOInputsAutoLogged();

    }
    
    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }
    
    //TODO: implement arm, wrist, and elevator constraints for real robot here (take from SuperVisualizer update)
    // public Command setAngle(double angle) {
    //  return Commands.run(()->{
    //     // why is it angle two? becuase thats what makes it stop yelling
    //     double angle2 = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    //     double currentAngle = inputs.positionRads-armZero;
    //     double output = armPID.calculate(currentAngle, angle2);
    //     io.runVolts(output);});
    // }
    
    public Command setVolts(double percent){
        return Commands.run(()->{
            target = percent;
            io.runVolts(percent);
        });
    }

   public void runVolt(double volt){
        io.runVolts(volt);
        target = volt;
   }

   public double getVel(){
    return inputs.velocityRadPerSec;
   }

   public double getVolt(){
    return inputs.appliedVolts;
   }
    
    // Commands need to be reviewed may have implemented them incorrectly
    public Command stopArm(Arm arm) {
        return Commands.run(()->{ io.stop();});
    }
    // The following commands will be used for going to 

    @AutoLogOutput(key = "Arm/Angle Radians")
    public  double getAngle() {
        return io.getAngle();
    }

    // @AutoLogOutput(key = "Arm/setpoint volt")
    // public  double getSetpoint() {
    //     return target;
    // }
    
    @AutoLogOutput(key = "Arm/isAtSetpoint")
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getAngle() - targetAngle) < tolerance;
    }
    
    public void resetAngleZero() {
        armZero = inputs.positionRads;
    }
}