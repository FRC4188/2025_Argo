package frc.robot.CSPLib.inputs;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Add your docs here. */
public class CSP_Controller extends CommandXboxController {
  public enum Scale {
    LINEAR,
    SQUARED,
    CUBED,
    QUARTIC
  }

  public CSP_Controller(int port) {
    super(port);
  }

  /**
   * Calculates joystick output to account for scale and deadband
   *
   * @param input input value
   * @param scale input scale
   * @return adjusted value
   */
  private static double getOutput(double input, Scale scale) {
    if (Math.abs(input) > Constants.controller.DEADBAND) {
      return scaleValue(input, scale);
    } else {
      return 0;
    }
  }

  private static double scaleValue(double input, Scale scale) {
    switch (scale) {
      case LINEAR:
        return input;
      case SQUARED:
        return Math.signum(input) * Math.pow(input, 2);
      case CUBED:
        return Math.pow(input, 3);
      case QUARTIC:
        return Math.signum(input) * Math.pow(input, 4);
      default:
        return input;
    }
  }

  // new corrected for circle deadband instead of square
  public Translation2d getCorrectedRight(Scale scale) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(super.getRightX(), super.getRightY()), Constants.controller.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(super.getRightY(), super.getRightX()));

    linearMagnitude = scaleValue(linearMagnitude, scale);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  // new corrected for circle deadband instead of square
  public Translation2d getCorrectedLeft(Scale scale) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(super.getLeftX(), super.getLeftY()), Constants.controller.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(super.getLeftY(), super.getLeftX()));

    linearMagnitude = scaleValue(linearMagnitude, scale);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * @param scale
   * @return
   */
  public double getRightY(Scale scale) {
    return getOutput(this.getRightY(), scale);
  }

  public double getRightX(Scale scale) {
    return getOutput(this.getRightX(), scale);
  }

  public double getLeftY(Scale scale) {
    return getOutput(this.getLeftY(), scale);
  }

  public double getLeftX(Scale scale) {
    return getOutput(this.getLeftX(), scale);
  }

  public Trigger getLeftS() {
    return this.leftStick();
  }

  public Trigger getRightS() {
    return this.rightStick();
  }

  public Trigger getXButton() {
    return this.x();
  }

  public Trigger getYButton() {
    return this.y();
  }

  public Trigger getAButton() {
    return this.a();
  }

  public Trigger getBButton() {
    return this.b();
  }

  public Trigger getUpButton() {
    return this.povUp();
  }

  public Trigger getDownButton() {
    return this.povDown();
  }

  public Trigger getRightButton() {
    return this.povRight();
  }

  public Trigger getLeftButton() {
    return this.povLeft();
  }

  public Trigger getLeftBumperButton() {
    return this.leftBumper();
  }

  public Trigger getRightBumperButton() {
    return this.rightBumper();
  }

  public Trigger getStartButton() {
    return this.start();
  }

  public Trigger getBackButton() {
    return this.back();
  }

  public double getRightT(Scale scale) {
    return getOutput(getRightTriggerAxis(), scale);
  }

  public double getLeftT(Scale scale) {
    return getOutput(getLeftTriggerAxis(), scale);
  }

  public Trigger getRightTButton() {
    return this.rightTrigger(0.1);
  }

  public Trigger getLeftTButton() {
    return this.leftTrigger(0.1);
  }
}
