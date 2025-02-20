package frc.robot.subsystems.scoring.intake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public static enum Mode {
        EMPTY,
        ALGAE,
        CORAL
    }

    //I need this to be global value - RN
    public static Mode intakeState = Mode.EMPTY;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    //TODO: create stall method
    public Command ingest(Mode intakeMode) {
        io.invertMotor(intakeMode == Mode.ALGAE);
        return Commands.run(
            () -> {
                io.runVolts(1);
            }).until(()-> io.isSafetyOn()).withTimeout(2).andThen(() -> intakeState = intakeMode);
    }

    public Command eject() {
        io.invertMotor(intakeState == Mode.CORAL);
        return Commands.run(
            () -> {
                io.runVolts(1);
                intakeState = Mode.EMPTY;
            }).withTimeout(1).andThen(() -> intakeState = Mode.EMPTY);
    }

    public Mode getState() {
        return intakeState;
    }

    public void setState(Mode state) {
        intakeState = state;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
