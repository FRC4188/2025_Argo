package frc.robot.subsystems.scoring.intake;


import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class Intake extends SubsystemBase{

    //I need this to be global value - RN

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
   

    public Intake(IntakeIO io){
        this.io = io;
        //DIO channel
        inputs = new IntakeIOInputsAutoLogged();
    }

    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    public Command ingest() {
        double voltage = 10;
        return Commands.runOnce(
            () -> {
                io.runVolts(voltage);
                //temporary
            });
    }

    public Command eject() {
        double voltage = -6;
        return Commands.runOnce(
            () -> {
                io.runVolts(voltage);
            });
    }

    public Command stop() {
        return Commands.runOnce(
            () -> {
                io.runVolts(0);
            });
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
