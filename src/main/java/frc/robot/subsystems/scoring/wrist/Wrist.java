
package frc.robot.subsystems.scoring.wrist;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.scoring.SuperConstraints;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Wrist extends SubsystemBase {//J.C
    private static Wrist instance;
    private WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();


    private SparkMax motor = new SparkMax(Constants.WristConstants.kWristId, MotorType.kBrushless);
    private ProfiledPIDController pid = new ProfiledPIDController(Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD, Constants.WristConstants.kConstraints);
    private RelativeEncoder encoder = motor.getEncoder();
    
    private double targetAngle = 0, wristAngle = 0;
    private final double appliedVolts = 0.0;
    private final double tempC = 0.0;
    private final double posRads = 0.0;
    private final double velRadsPerSec = 0.0;

    private Wrist(WristIO io){
      this.io = io;

    //   pid.reset(Constants.wrist.UPPER_LIMIT);
    //   pid.setTolerance(Constants.wrist.kTolerance);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    //TODO: fix conversion error ^
    Logger.processInputs("Wrist", inputs);    

    targetAngle = MathUtil.clamp(targetAngle, SuperConstraints.ArmConstraints.LOWEST_A, SuperConstraints.ArmConstraints.HIGHEST_A);
    wristAngle = pid.calculate(inputs.posRads, targetAngle);
    io.runVolts(wristAngle);
  }

  public Command setAngle(double angle) {
    return Commands.run(()->{
        targetAngle = angle;
    });
  }

  public Command runVolts(double volts) {
    return Commands.run(()->{
        io.runVolts(volts);
    });
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
    return Math.abs(getMotorAngle() - angle) < Constants.WristConstants.kTolerance;
  }
  public double getVelocity() {
    return inputs.velRadsPerSec;
  }

  public double getPosition() {
    return inputs.posRads;
  }

  public double getMotorAngle() {
    return inputs.posRads * Constants.WristConstants.kDegree_per_rads;
  }

  public double getMotorVoltage(){
    return inputs.appliedVolts;
  }

  public double getMotorTemperature() {
    return inputs.tempC;
  }
}
