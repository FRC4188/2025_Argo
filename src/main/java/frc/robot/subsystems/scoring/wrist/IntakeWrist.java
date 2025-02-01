package frc.robot.subsystems.scoring.wrist;


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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
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
  
  public Command setWristAngle(double angle){
    return Commands.run(()-> {
      setAngle(angle);
    });
  }
 
  
  
  
  

}