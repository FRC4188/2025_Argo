package frc.robot.subsystems.scoring.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX motor;
    
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final DigitalInput breaker;

    public IntakeIOReal(){
        motor = new TalonFX(Constants.Id.kIntake, Constants.robot.rio);
        breaker = new DigitalInput(0);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(IntakeConstants.kMotorConfig);
        appliedVolts = motor.getMotorVoltage();
        tempC = motor.getDeviceTemp();

        appliedVolts.setUpdateFrequency(Hertz.of(50));
        tempC.setUpdateFrequency(Hertz.of(0.5));

        motor.optimizeBusUtilization();
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

 public boolean isIn(){
        return !breaker.get(); //true when laser can hit breaker aka nothing in intake
    }

    @Override
    public boolean isStalled() {
        return  (Math.abs(motor.getStatorCurrent().getValueAsDouble()) > 35);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
       
    }
    
}