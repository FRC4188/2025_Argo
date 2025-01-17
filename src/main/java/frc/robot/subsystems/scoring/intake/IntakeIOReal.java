package frc.robot.subsystems.scoring.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged.Naming;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private TalonFX motor;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private final StatusSignal<AngularVelocity> velRadsPerSec;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOReal(){
        motor = new TalonFX(Constants.ids.INTAKE, "rio");
        motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        appliedVolts = motor.getMotorVoltage();
        tempC = motor.getDeviceTemp();
        posRads = motor.getPosition();
        velRadsPerSec = motor.getVelocity();

        //signal updates less frequently since intake is less important, and to reduce CAN bus traffic
        //intake acc doesnt need status signal at all, i included it for options so we can monitor voltage usage
        BaseStatusSignal.setUpdateFrequencyForAll(
            Frequency.ofRelativeUnits(10.0,
            Units.Hertz), 
            appliedVolts,
            tempC,
            posRads,
            velRadsPerSec);
        
        motor.optimizeBusUtilization();
    }


    @Override
    public void runVolts(double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.posRads = posRads.getValueAsDouble();
        inputs.velRadsPerSec = velRadsPerSec.getValueAsDouble();
    }
    
}
