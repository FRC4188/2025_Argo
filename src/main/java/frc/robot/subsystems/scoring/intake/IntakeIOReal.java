package frc.robot.subsystems.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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