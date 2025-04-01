
package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public double kZero = 0;

    public Elevator(ElevatorIO io){
        this.io = io;
        inputs = new ElevatorIOInputsAutoLogged();
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);   

        // if (io.isStalled()) setZero();
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void setZero() {
        kZero = io.getHeight();
      }


    @AutoLogOutput(key = "Elevator/Height Meters")
    public double getHeight(){
        return io.getHeight() - kZero;
    }

    public boolean atGoal(double target) {
        return Math.abs(getHeight() - target) < ElevatorConstants.kTolerance;
    }

    public Command runSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds),
                Volts.of(8),
                Seconds.of(6)
            ),new SysIdRoutine.Mechanism(
                voltage -> io.runVolts(voltage.magnitude()),
                state -> SignalLogger.writeString("SysId_Elevator", state.toString()),
                this));
        
        return Commands.sequence(
            routine.quasistatic(Direction.kForward)
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.quasistatic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0),
                
            routine.dynamic(Direction.kForward)
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.dynamic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0)
        );
    }
}
