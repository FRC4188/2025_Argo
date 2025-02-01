// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package CSP_Lib.utils;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

import java.util.ArrayList;

import CSP_Lib.Motors.CSP_Motor;

/** Add your docs here. */
public final class TempManager {
  private static ArrayList<CSP_Motor> motorsList = new ArrayList<CSP_Motor>();

  /**
   * Add motors to list of motors to be monitored
   *
   * @param motors
   */
  public static void addMotor(CSP_Motor... motors) {
    for (CSP_Motor motor : motors) {
      motorsList.add(motor);
    }
  }

  /** Begin monitoring */
  public static void monitor() {
    for (CSP_Motor motor : motorsList) {
      if (motor.getTemperature() > Constants.wrist.MAX_TEMP
          && motor.getTemperature() < Constants.wrist.MAX_TEMP + 10) {
        String tempWarning =
            "Motor " + motor.getID() + " Maximum Temperature Reached " + motor.getTemperature();
        DriverStation.reportWarning(tempWarning, false);
      } else if (motor.getTemperature() > Constants.wrist.MAX_TEMP + 10) {
        String tempError =
            "Motor " + motor.getID() + " Critical Temperature Reached " + motor.getTemperature();
        DriverStation.reportError(tempError, false);
      }
    }
  }
}