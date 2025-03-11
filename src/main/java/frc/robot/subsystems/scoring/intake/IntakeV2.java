package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.intake.Intake.Mode;

public class IntakeV2 extends SubsystemBase{
    //basically just got rid of all instances of code that depended on whether or not it was coral or algae intake

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public IntakeV2(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public Command ingest(){
        double voltage = 3;
        return Commands.runOnce(
            () -> {
                io.runVolts(voltage);
            });
    }

    public Command eject() {
        double voltage = -3;
        return Commands.runOnce(() -> {
            io.runVolts(voltage);
        });
    }

    public Command stop(){
        return Commands.runOnce(() -> {
            io.runVolts(0);
        });
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
        


}
