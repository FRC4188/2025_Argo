package frc.robot.subsystems.scoring.intake;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    public Command ingest(Mode intakeMode) {
        double voltage = (intakeMode == Mode.ALGAE)? 3:-3;
        return Commands.run(
            () -> {
                io.runVolts(voltage);
            }).until(()-> io.isStalled()).andThen(() -> io.runVolts(0.05)).andThen(() -> intakeState = intakeMode);
    }

    public Command eject() {
        double voltage = (intakeState == Mode.CORAL)? 3:-3;
        return Commands.run(
            () -> {
                io.runVolts(voltage);
                intakeState = Mode.EMPTY;
            })
            .withTimeout(1)
            .andThen(() -> io.runVolts(0)).andThen(() -> intakeState = Mode.EMPTY);
    }

    public Command stop() {
        return Commands.runOnce(() -> io.runVolts(0));
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

        String state;
        if (intakeState == Mode.CORAL) {
            state = "coral";
        } else if (intakeState == Mode.ALGAE) {
            state = "algae";
        } else {
            state = "empty";
        }
        Logger.recordOutput("Intake/State", state);
    }
}
