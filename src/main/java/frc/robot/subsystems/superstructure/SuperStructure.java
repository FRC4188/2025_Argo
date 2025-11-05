package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.elevator.*;
import frc.robot.subsystems.superstructure.wrist.*;

public class SuperStructure extends SubsystemBase {

  private final Elevator elevator;
  private final Wrist wrist;

  public SuperStructure(ElevatorIO eleIO, WristIO wristIO) {
    elevator = new Elevator(eleIO);
    wrist = new Wrist(wristIO);
  }

  @Override
  public void periodic() {
    elevator.periodic();
    wrist.periodic();

    if (Constants.pid_mode == Constants.PIDTuning.WRIST) {
      wrist.setPosition(Rotation2d.kZero);
    }
  }

  public void resetElevator() {
    elevator.setZero();
  }

  public void runWrist(double volts) {
    wrist.runVolts(volts);
  }

  public void runElevator(double volts) {
    elevator.runVolts(volts);
  }

  public void setState(SuperState state) {
    setWrist(state.getWristAngle());
    setElevator(state.getEleHeight());
  }

  public void setWrist(Rotation2d radians) {
    wrist.setPosition(radians);
  }

  public void setElevator(double meters) {
    elevator.setHeight(meters);
  }

  public boolean atStateGoal(SuperState goal, double eleTol, double wristTol) {
    return atWristGoal(goal.getWristAngle(), wristTol)
        && atElevatorGoal(goal.getEleHeight(), eleTol);
  }

  public boolean atWristGoal(Rotation2d radians, double tolerance) {
    return Math.abs(wrist.getAngle().getRadians() - radians.getRadians()) < tolerance;
  }

  public boolean atElevatorGoal(double meters, double tolerance) {
    return Math.abs(elevator.getPositionMeters() - meters) < tolerance;
  }

  public boolean atStateGoal(SuperState goal) {
    return atWristGoal(goal.getWristAngle()) && atElevatorGoal(goal.getEleHeight());
  }

  public boolean atWristGoal(Rotation2d radians) {
    return Math.abs(wrist.getAngle().getRadians() - radians.getRadians())
        < Constants.WristConstants.kTolerance;
  }

  public boolean atElevatorGoal(double meters) {
    return Math.abs(elevator.getPositionMeters() - meters) < Constants.EleConstants.kTolerance;
  }

  public void updateElePID() {
    elevator.updatePID();
  }

  public void updateWristPID() {
    wrist.updatePID();
  }

  public SuperState getState() {
    return new SuperState(wrist.getAngle().getRadians(), elevator.getPositionMeters());
  }
}
