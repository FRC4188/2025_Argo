
package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    ElevatorFeedforward ff;

    public Elevator(ElevatorIO io){
        this.io = io;
        inputs = new ElevatorIOInputsAutoLogged();
        ff = new ElevatorFeedforward(
            kMotorConfig.Slot0.kS, 
            kMotorConfig.Slot0.kS, 
            kMotorConfig.Slot0.kS); //TODO: get values from sysid
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);   

    }

    @AutoLogOutput(key = "Elevator/Height Meters")
    public double getHeight(){
        return io.getHeight();
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
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
