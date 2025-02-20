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
        System.out.println("ingesting algae " + (intakeMode == Mode.ALGAE));
        io.invertMotor(intakeMode == Mode.ALGAE);
        return Commands.run(
            () -> {
                io.runVolts(1);
            }).until(()-> io.isSafetyOn()).andThen(() -> intakeState = intakeMode).andThen(() -> System.out.println("ingested algae: " + (intakeState == Mode.ALGAE)));
    }

    public Command eject() {
        System.out.println("ejecting");
        io.invertMotor(intakeState == Mode.CORAL);
        return Commands.run(
            () -> {
                io.runVolts(1);
            }).andThen(() -> intakeState = Mode.EMPTY).andThen(() -> System.out.println("ejected"));
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
