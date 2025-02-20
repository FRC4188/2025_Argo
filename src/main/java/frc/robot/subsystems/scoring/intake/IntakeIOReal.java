package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private WPI_TalonSRX motor;

    private final double appliedVolts;
    private final double tempC;

    public IntakeIOReal(){
        motor = new WPI_TalonSRX(Constants.Id.kIntake);

        appliedVolts = motor.getMotorOutputVoltage();
        tempC = motor.getTemperature();
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
        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
    }
    
}