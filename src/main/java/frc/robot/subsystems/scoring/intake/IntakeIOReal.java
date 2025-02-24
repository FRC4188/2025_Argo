package frc.robot.subsystems.scoring.intake;

import static edu.wpi.first.units.Units.Hertz;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX motor;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;

    public IntakeIOReal(){
        motor = new TalonFX(Constants.Id.kIntake, Constants.robot.rio);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(IntakeConstants.kMotorConfig);
        motor.optimizeBusUtilization();

        appliedVolts = motor.getMotorVoltage();
        tempC = motor.getDeviceTemp();

        appliedVolts.setUpdateFrequency(Hertz.of(50));
        tempC.setUpdateFrequency(Hertz.of(0.5));
    }

    @AutoLogOutput(key = "Intake/Safety On")
    public boolean stalling() {
        return motor.getVelocity().getValueAsDouble() < 1;
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