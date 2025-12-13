package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leaderTalon;
  private final TalonFX followTalon;

  private final TalonFXConfiguration elevatorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Voltage> followAppliedVolts;
  private final StatusSignal<Current> followCurrent;

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOReal() {
    leaderTalon = new TalonFX(Constants.Id.kElevatorLead, Constants.robot.rio);
    followTalon = new TalonFX(Constants.Id.kElevatorFollow, Constants.robot.rio);

    followTalon.setControl(new Follower(Constants.Id.kElevatorLead, false));

    elevatorConfig = Constants.EleConstants.kInitialConfigs;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorConfig.Slot0 = Constants.EleConstants.motorGains;
    elevatorConfig.Feedback.SensorToMechanismRatio = Constants.EleConstants.kGearRatio;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / Constants.EleConstants.kGearRatio;
    elevatorConfig.MotionMagic.MotionMagicAcceleration =
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.EleConstants.kGearRatio;
    elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    elevatorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    elevatorConfig.MotorOutput.Inverted =
        Constants.EleConstants.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> followTalon.getConfigurator().apply(elevatorConfig, 0.25));

    position = leaderTalon.getPosition();
    velocity = leaderTalon.getVelocity();
    leaderAppliedVolts = leaderTalon.getMotorVoltage();
    leaderCurrent = leaderTalon.getStatorCurrent();
    followAppliedVolts = followTalon.getMotorVoltage();
    followCurrent = followTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, leaderAppliedVolts, leaderCurrent, followAppliedVolts, followCurrent);
    ParentDevice.optimizeBusUtilizationForAll(leaderTalon, followTalon);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(position, velocity, leaderAppliedVolts, leaderCurrent);
    var followStatus = BaseStatusSignal.refreshAll(followAppliedVolts, followCurrent);

    inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();

    inputs.followConnected = followConnectedDebounce.calculate(followStatus.isOK());
    inputs.followAppliedVolts = followAppliedVolts.getValueAsDouble();
    inputs.followCurrentAmps = followCurrent.getValueAsDouble();
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kG) {
    elevatorConfig.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKG(kG)
            .withGravityType(GravityTypeValue.Elevator_Static);
    tryUntilOk(5, () -> leaderTalon.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> followTalon.getConfigurator().apply(elevatorConfig, 0.25));
  }

  @Override
  public void setOpenLoop(double output) {
    leaderTalon.setControl(
        switch (Constants.EleConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setZero() {
    leaderTalon.setPosition(0);
  }

  @Override
  public double getSetpoint() {
    return leaderTalon.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    leaderTalon.setControl(
        switch (Constants.EleConstants.motorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }
}
