package frc.robot.subsystems.scoring.intake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public enum Mode {
        EMPTY,
        ALGAE,
        CORAL
    }

    //I need this to be global value - RN
    public static Mode intakeState = Mode.EMPTY;
    private Mode intakeMode = Mode.CORAL;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public void setMode(Mode mode) {
        intakeMode = mode;
    }

    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    //TODO: create stall method
    public Command ingest() {
        return Commands.run(
            () -> {
                io.invertMotor(intakeMode == Mode.ALGAE);
                io.runVolts(1);
            }).until(()-> io.isSafetyOn());
    }

    public Command eject() {
        return Commands.run(
            () -> {
                io.invertMotor(intakeMode == Mode.CORAL);
                io.runVolts(1);
            });
    }

    public Mode getState() {
        return intakeState;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
