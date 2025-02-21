package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private TalonFX motor;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;

    public IntakeIOReal(){
        motor = new TalonFX(Constants.Id.kIntake);

        appliedVolts = motor.getMotorVoltage();
        tempC = motor.getDeviceTemp();
    }


    @AutoLogOutput(key = "Intake/Safety On")
    public boolean isSafetyOn() {
        return motor.isSafetyEnabled();
    }

    @AutoLogOutput(key = "Intake/Is Alive")
    public boolean isAlive() {
        return motor.isAlive();
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
    }
    
}