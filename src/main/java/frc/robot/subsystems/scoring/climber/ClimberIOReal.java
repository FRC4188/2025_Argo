package frc.robot.subsystems.scoring.climber;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
    private final TalonFX motor;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRots;

    public ClimberIOReal() {
        motor = new TalonFX(Constants.Id.kClimber, Constants.robot.rio); 
        
        motor.clearStickyFaults();
        motor.getConfigurator().apply(Constants.ClimberConstants.kMotorConfig);
        motor.optimizeBusUtilization();  
        
        posRots = motor.getPosition();
        appliedVolts = motor.getMotorVoltage();
        tempC = motor.getDeviceTemp();

        posRots.setUpdateFrequency(Hertz.of(50));
        appliedVolts.setUpdateFrequency(Hertz.of(50));
        tempC.setUpdateFrequency(Hertz.of(0.5));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                posRots, appliedVolts, tempC).isOK();

        inputs.posRads = Units.rotationsToRadians(posRots.getValueAsDouble());
        inputs.tempC = tempC.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();        

    }

    public void runVolts(double volts) {
        MathUtil.clamp(volts, -12, 12);
        motor.setVoltage(volts);
    }

    public double getAngle() {
        return Units.rotationsToRadians(posRots.getValueAsDouble());
    }
    
}
