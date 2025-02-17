
package frc.robot.subsystems.scoring.wrist;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;


public class Wrist extends SubsystemBase {//J.C
  private WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io){
    this.io = io;
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);    
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  @AutoLogOutput(key = "Wrist/Angle Radians")
  public double getAngle(){
    return io.getAngle();
  }

  public double getVolt(){
    return inputs.appliedVolts;
  }

  @AutoLogOutput(key = "Wrist/isAtGoal")
  public boolean atGoal(double goal) {
    return Math.abs(getAngle() - goal) < Constants.WristConstants.kTolerance;
  }
}
