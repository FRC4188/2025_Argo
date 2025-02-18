package frc.robot.subsystems.scoring.arm;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor;
    private final CANcoder armEncoder;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private final StatusSignal<Angle> desiredPos;

    public ArmIOReal() {
        armMotor = new TalonFX(Constants.Id.kArm);
        armEncoder = new CANcoder(Constants.Id.kArmNcoder);
        //TODO: encoder config? (which includes zero offset)

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.getConfigurator().apply(ArmConstants.kMotorConfig);
        armMotor.optimizeBusUtilization();

        appliedVolts = armMotor.getMotorVoltage();
        tempC = armMotor.getDeviceTemp();
        posRads = armMotor.getPosition();
        desiredPos = armEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50), 
            appliedVolts,
            posRads)    ;
    }

    @Override
    public void runVolts(double volts) {
        volts = MathUtil.clamp(volts,-12, 12);
        armMotor.set(volts);
    }

    @Override
    public void stop() {
        runVolts(0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(DriverStation.isDisabled()){
            runVolts(0.0);
        }

        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.positionRads = posRads.getValueAsDouble();
        inputs.desiredPositionRads = desiredPos.getValueAsDouble();
    }

    @Override
    public double getAngle(){
        return posRads.getValueAsDouble();
    }
}