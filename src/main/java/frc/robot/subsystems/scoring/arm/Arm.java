package frc.robot.subsystems.scoring.arm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    public Arm(ArmIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

   public void runVolt(double volt){
        io.runVolts(volt);
   }

   public double getVolt(){
    return inputs.appliedVolts;
   }


    @AutoLogOutput(key = "Arm/Angle Radians")
    public  double getAngle() {
        return io.getAngle();
    }
    
    @AutoLogOutput(key = "Arm/isAtSetpoint")
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getAngle() - targetAngle) < Constants.ArmConstants.kTolerance;
    }
}