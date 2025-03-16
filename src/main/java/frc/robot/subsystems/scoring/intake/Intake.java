package frc.robot.subsystems.scoring.intake;


import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //I need this to be global value - RN

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    private final DigitalInput breaker;

    public Intake(IntakeIO io){
        this.io = io;
        this.breaker = new DigitalInput(0); //DIO channel
        inputs = new IntakeIOInputsAutoLogged();
    }

    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    public Command ingest(boolean isSlow) {
        double voltage = isSlow? 6: 12;
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
                //temporary
            });
    }

    public boolean isIn(){
        return !breaker.get(); //true when laser can hit breaker aka nothing in intake
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
