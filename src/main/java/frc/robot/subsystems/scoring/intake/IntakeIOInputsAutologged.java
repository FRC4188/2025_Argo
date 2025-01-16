package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.scoring.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOInputsAutologged extends IntakeIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Applied Volts", appliedVolts);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        appliedVolts = table.get("Applied Volts", appliedVolts);
    }
    
}
