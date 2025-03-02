package frc.robot.subsystems.scoring.arm;

import static edu.wpi.first.units.Units.Hertz;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.autos.pathgen.PG_math;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor;
    private final DutyCycleEncoder armEncoder;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private double encoderRad;

    public ArmIOReal() {
        armMotor = new TalonFX(Constants.Id.kArm, Constants.robot.rio);
        armEncoder = new DutyCycleEncoder(3, 19.8019801, 0);

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.getConfigurator().apply(ArmConstants.kMotorConfig);

        appliedVolts = armMotor.getMotorVoltage();
        tempC = armMotor.getDeviceTemp();
        posRads = armMotor.getPosition();
        encoderRad = Units.rotationsToRadians(armEncoder.get());

        appliedVolts.setUpdateFrequency(Hertz.of(50));
        tempC.setUpdateFrequency(Hertz.of(0.5));
        posRads.setUpdateFrequency(Hertz.of(50));
        armEncoder.setAssumedFrequency(50);

        armMotor.optimizeBusUtilization();
    }

    @Override
    public void runVolts(double volts) {
        volts = MathUtil.clamp(volts,-12, 12);
        armMotor.setVoltage(-volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(DriverStation.isDisabled()){
            runVolts(0.0);
        }
        encoderRad = -PG_math.modulate(Rotation2d.fromRotations(armEncoder.get()).minus(Rotation2d.fromRadians(ArmConstants.kZero))).getRadians();

        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.positionRads = posRads.getValueAsDouble();
        inputs.desiredPositionRads = encoderRad;
    }

    @Override
    public double getAngle(){
        return -PG_math.modulate(Rotation2d.fromRotations(armEncoder.get()).minus(Rotation2d.fromRadians(ArmConstants.kZero))).getRadians();
    }
}