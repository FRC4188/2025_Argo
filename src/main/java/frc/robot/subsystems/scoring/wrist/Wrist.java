
package frc.robot.subsystems.scoring.wrist;


import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
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
  public Command runSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(4).per(Seconds),
                Volts.of(8),
                Seconds.of(6)
            ),new SysIdRoutine.Mechanism(
                voltage -> runVolts(voltage.magnitude()),
                null,
                this));
        
        return Commands.sequence(
            routine.quasistatic(Direction.kForward)
                .until(() -> getAngle() >= SuperConstraints.WristConstraints.HIGHEST_A),
            routine.quasistatic(Direction.kReverse)
                .until(() -> getAngle() <= SuperConstraints.WristConstraints.LOWEST_A),
                
            routine.dynamic(Direction.kForward)
                .until(() -> getAngle() >= SuperConstraints.WristConstraints.HIGHEST_A),
            routine.dynamic(Direction.kReverse)
                .until(() -> getAngle() <= SuperConstraints.WristConstraints.LOWEST_A)
        );
    }
}