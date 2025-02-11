
package frc.robot.subsystems.scoring.wrist;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.scoring.SuperConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Wrist extends SubsystemBase {//J.C
    private WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();


    // private ProfiledPIDController pid = new ProfiledPIDController(Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD, Constants.WristConstants.kConstraints);
    
    private double targetAngle = 0, wristAngle = 0;

    private Wrist(WristIO io){
      this.io = io;

    //   pid.reset(Constants.wrist.UPPER_LIMIT);
    //   pid.setTolerance(Constants.wrist.kTolerance);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    //TODO: fix conversion error ^
    Logger.processInputs("Wrist", inputs);    

    // wristAngle = pid.calculate(inputs.posRads, targetAngle);
    wristAngle = inputs.posRads * Math.PI / 180;
    io.runVolts(targetAngle * Math.PI / 180);
  }

  public Command setAngle(double angle) {
    return Commands.run(()->{
        targetAngle = angle;
        targetAngle = MathUtil.clamp(targetAngle, SuperConstraints.ArmConstraints.LOWEST_A, SuperConstraints.ArmConstraints.HIGHEST_A);
    });
  }

  public Command runVolts(double volts) {
    return Commands.run(()->{
        io.runVolts(volts);
    });
  }

//   public void setPID(double kP, double kI, double kD) {
//     pid.setPID(kP, kI, kD);
//   }

  // public double getAngle() {
  //   return encoder.getAbsolutePosition();
  // }

  @AutoLogOutput(key = "Wrist/Setpoint")
  public double getSetpoint() {
    return targetAngle;
  }

  @AutoLogOutput(key = "Wrist/isAtGoal")
  public boolean atGoal() {
    return Math.abs(wristAngle - targetAngle) < Constants.WristConstants.kTolerance;
  }
}
