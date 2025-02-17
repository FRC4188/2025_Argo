package frc.robot.subsystems.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private WPI_TalonSRX motor;

    private final double appliedVolts;
    private final double tempC;
    // private final StatusSignal<Angle> posRads;
    // private final StatusSignal<AngularVelocity> velRadsPerSec;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOReal(){
        motor = new WPI_TalonSRX(Constants.Id.kIntake);
        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        motor.setSafetyEnabled(false);

        appliedVolts = motor.getMotorOutputVoltage();
        tempC = motor.getTemperature();
        // posRads = encoder.getPosition();
        // velRadsPerSec = encoder.getVelocity();
    }

    // might keep intake permanently stopped 
    // since safety will make it impossible to outtake
    public void stopIfSafety() {
        if (motor.isSafetyEnabled() == true) {
            stop();
        }        
    }

    public boolean isSafetyOn(boolean isSafe) {
        if (motor.isSafetyEnabled() == true) {
            isSafe = true;
            return isSafe;
        } else {
            isSafe = false;
            return isSafe;
        }
    }

    @Override
    public void runVolts(double volts) {
        // motor.set(voltageOut.withOutput(volts));
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
        // inputs.posRads = posRads.getValueAsDouble();
        // inputs.velRadsPerSec = velRadsPerSec.getValueAsDouble();
    }
    
}