package frc.robot.subsystems.scoring.intake;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class Intake extends SubsystemBase{

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public Command ingest(DoubleSupplier volts) {
        return Commands.runEnd(
            () -> 
                io.runVolts(volts.getAsDouble()),
            ()-> io.stop()
            ,this);
    }

    public Command eject(DoubleSupplier volts) {
        return Commands.runEnd(
            () -> {
                io.runVolts(-volts.getAsDouble());
            },
            () -> io.stop()
            ,this);
    }

    public Command stop() {
        return Commands.runOnce(
            () -> {
                io.runVolts(0);
            },this);
    }

    @AutoLogOutput(key = "Intake/Algae is In?")
    public boolean isIn() {
        return io.isIn();
    }

    @AutoLogOutput(key = "Intake/Is Stalled?")
    public boolean isStalled() {
        return io.isStalled();
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);   
        
    }
}
