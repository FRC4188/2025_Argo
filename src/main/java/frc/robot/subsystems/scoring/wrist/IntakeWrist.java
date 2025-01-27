package frc.robot.subsystems.scoring.wrist;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.scoring.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.scoring.wrist.IntakeWristIOReal;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeWrist extends SubsystemBase {//J.C
    private static IntakeWrist instance;
    private IntakeWristIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();


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

      pid.reset(Constants.wrist.UPPER_LIMIT);
      pid.setTolerance(Constants.wrist.ALLOWED_ERROR);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);    
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

  @AutoLogOutput(key = "Wrist/Setpoint")
  public double getSetpoint() {
    return pid.getSetpoint().position;
  }

  public boolean atGoal(double angle) {
    return Math.abs(getMotorAngle() - angle) < Constants.wrist.ALLOWED_ERROR;
  }
  public double getVelocity() {
    return inputs.velRadsPerSec;
  }

  public double getPosition() {
    return inputs.posRads;
  }

  public double getMotorAngle() {
    return inputs.posRads * Constants.wrist.WRIST_DEGREES_PER_MOTOR_ROTATION;
  }

  public double getMotorVoltage(){
    return inputs.appliedVolts;
  }

  public double getMotorTemperature() {
    return inputs.tempC;
  }
  
  
  
  

}