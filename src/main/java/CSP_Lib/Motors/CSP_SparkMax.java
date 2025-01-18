package CSP_Lib.Motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import CSP_Lib.utils.TempManager;

import com.revrobotics.RelativeEncoder;


public class CSP_SparkMax extends SparkMax implements CSP_Motor {
    SparkMax motor = new SparkMax(1, MotorType.kBrushless);

    boolean isInverted = motor.configAccessor.getInverted();
    double setPositionConversionFactor =motor.configAccessor.encoder.getPositionConversionFactor();
    double velocityFactor = motor.configAccessor.encoder.getVelocityConversionFactor();

    private SparkClosedLoopController pid;

    
    private RelativeEncoder encoder;
    SparkMaxConfig config = new SparkMaxConfig();
  

  public CSP_SparkMax(int id, MotorType kbrushless) {
    super(id, MotorType.kBrushless);
    encoder = getEncoder();
    init();
    
    TempManager.addMotor(this);
  }


  
public void init() {

    super.clearFaults();
    setEncoder(0);
  }

  public void set(double percent) {
    super.set(percent);
  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
  }

    public void setScalar(double scalar) {
        super.set(scalar);
    }
  



    

  public void setEncoder(double position) {
    encoder.setPosition(position / (2.0 * Math.PI));
  }

  public double getRPM() {
    return encoder.getVelocity();
  }
  public double getPosition() {
    return encoder.getPosition();
  }

  public double getPositionDegrees() {
    return encoder.getPosition() * 360;
  }

  public double getTemperature() {
    return super.getMotorTemperature();
  }

  public double getCurrent() {
    return super.getOutputCurrent();
  }

  public int getID() {
    return super.getDeviceId();
  }

@Override
    public void setBrake(boolean braking) {
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }



    @Override
    public void setRampRate(double rate) {
        config.openLoopRampRate(rate);
        configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}