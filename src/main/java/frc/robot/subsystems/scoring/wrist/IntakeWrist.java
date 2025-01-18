package frc.robot.subsystems.scoring.wrist;


import com.ctre.phoenix6.BaseStatusSignal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.scoring.wrist.IntakeWristIOReal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.wrist.IntakeWristIO;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.intake.IntakeIOInputsAutoLogged;

public class IntakeWrist extends SubsystemBase {
    private static IntakeWrist instance;
        private IntakeWristIO io;
    private final IntakeIOInputsAutoLogged inputs;


    private SparkMax motor = new SparkMax(Constants.wrist.WRIST, MotorType.kBrushless);
    private ProfiledPIDController pid = new ProfiledPIDController(Constants.wrist.kP, Constants.wrist.kI, Constants.wrist.kD, Constants.wrist.CONSTRAINTS);
    private RelativeEncoder encoder = motor.getEncoder();
    
    private double setpoint = 0;
    private final double appliedVolts = 0.0;
    private final double tempC = 0.0;
    private final double posRads = 0.0;
    private final double velRadsPerSec = 0.0;

    public static IntakeWrist getInstance(IntakeWristIOReal io) {
        if(instance == null){
            instance = new IntakeWrist(io);
        }
        return instance;
    }
    private IntakeWrist(IntakeWristIO io){
      this.io = io;
      inputs = new IntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Wrist Angle", getMotorAngle());
    SmartDashboard.putNumber("Wrist Setpoint", getSetpoint());
    SmartDashboard.putNumber("Applied Volts", appliedVolts);
    SmartDashboard.putNumber("Temperature(C)", tempC);
    SmartDashboard.putNumber("Position(Rads)", posRads);
    SmartDashboard.putNumber("Velocity(RadsPerSec)", velRadsPerSec);
  }

  private void init() {
    pid.reset(Constants.ids.UPPER_LIMIT);
    pid.setTolerance(Constants.ids.ALLOWED_ERROR);
  }

  public void disable() {
    motor.disable();
  }

  public void set(double percent) {
    motor.set(percent);
  }

  public void setAngle(double angle) {
    motor.set(pid.calculate(getMotorAngle(), angle));
  }

  public void setPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  // public double getAngle() {
  //   return encoder.getAbsolutePosition();
  // }

  public double getSetpoint() {
    return pid.getSetpoint().position;
  }

  public boolean atGoal(double angle) {
    return Math.abs(getMotorAngle() - angle) < Constants.ids.ALLOWED_ERROR;
  }
  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public int getID() {
    return motor.getDeviceId();
  }

  public double getMotorAngle() {
    return encoder.getPosition() * Constants.wrist.WRIST_DEGREES_PER_MOTOR_ROTATION;
  }


  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  public double getMotorVoltage(){
    return motor.getAppliedOutput();
  }

  public double getMotorTemperature() {
    return motor.getMotorTemperature();
  }
  public double setMotorTemperature() {
    return motor.getMotorTemperature();
  }
  public static void optimizeBusUtilization() {

  }
  
      
  

}