// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class WristIOReal implements WristIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase wristSpark;
  private final RelativeEncoder wristEncoder;
  private final CANcoder cancoder;

  // wrist config
  private SparkMaxConfig wristConfig;

  // Closed loop controllers
  private final SparkClosedLoopController wristController;
  private final ArmFeedforward wristff;

  // status singals for CTRE stuff
  private final StatusSignal<Angle> wristAbsolutePosition;

  private final Debouncer wristConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer cancoderConnectedDebounce = new Debouncer(0.5);

  public WristIOReal() {

    zeroRotation = new Rotation2d();

    wristSpark = new SparkMax(Constants.Id.kWrist, MotorType.kBrushless);
    wristEncoder = wristSpark.getEncoder();
    wristController = wristSpark.getClosedLoopController();
    wristff =
        new ArmFeedforward(
            Constants.WristConstants.kS, Constants.WristConstants.kG, Constants.WristConstants.kV);

    cancoder = new CANcoder(Constants.Id.kWristCANCoder, Constants.robot.rio);

    // Configure turn motor
    wristConfig = new SparkMaxConfig();
    wristConfig
        .inverted(Constants.WristConstants.kSparkInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.WristConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    wristConfig
        .encoder
        // .inverted(Constants.WristConstants.kEncoderInverted)
        .positionConversionFactor(Constants.WristConstants.kEncoderPositionFactor)
        .velocityConversionFactor(Constants.WristConstants.kEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    wristConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            Constants.WristConstants.kPIDMinInput, Constants.WristConstants.kPIDMaxInput)
        .pidf(Constants.WristConstants.kP, 0.0, Constants.WristConstants.kD, 0.0);
    wristConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        wristSpark,
        5,
        () ->
            wristSpark.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = Constants.WristConstants.kEncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        Constants.WristConstants.kEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    wristAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, wristAbsolutePosition);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        wristSpark,
        wristEncoder::getPosition,
        (value) -> inputs.wristPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(wristSpark, wristEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        wristSpark,
        new DoubleSupplier[] {wristSpark::getAppliedOutput, wristSpark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(wristSpark, wristSpark::getOutputCurrent, (value) -> inputs.currentAmps = value);

    inputs.wristConnected = wristConnectedDebounce.calculate(!sparkStickyFault);
    var wristEncoderStatus = BaseStatusSignal.refreshAll(wristAbsolutePosition);
    inputs.cancoderConnected = cancoderConnectedDebounce.calculate(wristEncoderStatus.isOK());

    inputs.wristAbsolutePosition =
        Rotation2d.fromRotations(wristAbsolutePosition.getValueAsDouble());

    if (inputs.cancoderConnected) {
      wristEncoder.setPosition(inputs.wristAbsolutePosition.getRadians());
    }
  }

  @Override
  public void setOpenLoop(double output) {
    wristSpark.setVoltage(output);
  }

  @Override
  public void setPosition(Rotation2d rotation) {

    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            Constants.WristConstants.kPIDMinInput,
            Constants.WristConstants.kPIDMaxInput);
    wristController.setReference(
        setpoint,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        wristff.calculate(
            Units.rotationsToRadians((Math.PI / 2) - wristAbsolutePosition.getValueAsDouble()), 0));
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kG) {
    wristConfig.closedLoop.pidf(kP, kI, kD, 0);
    wristff.setKg(kG);
  }
}
