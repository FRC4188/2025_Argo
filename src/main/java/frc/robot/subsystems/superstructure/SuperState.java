package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SuperState {

  // radians and meters
  private Rotation2d wrist_angle;
  private double elevator_height;

  public SuperState() {
    wrist_angle = new Rotation2d();
    elevator_height = 0;
  }

  public SuperState(double radians, double elevator) {
    wrist_angle = Rotation2d.fromRadians(radians);
    elevator_height = elevator;
  }

  public Rotation2d getWristAngle() {
    return wrist_angle;
  }

  public double getEleHeight() {
    return elevator_height;
  }

  public static enum SuperPreset {
    L3_ALGAE(new SuperState(0.0, 0.85)),

    L2_ALGAE(new SuperState(0.0, 0.45)),

    L1_CORAL(new SuperState(0, 0.4719 - Units.inchesToMeters(6))),

    NET(new SuperState(1, Units.inchesToMeters(72))),

    ALGAE_GROUND(new SuperState(1, 0)),

    START(new SuperState(0, 0)),

    PROCESSOR(new SuperState(0.5, 0)),

    ALGAE_STOW(PROCESSOR.getState());
    // Note that processor and stow are the same thing, just separate variables for naming purposes

    private final SuperState state;

    private SuperPreset(SuperState state) {
      this.state = state;
    }

    public SuperState getState() {
      return state;
    }
  }

  public String toString() {
    return String.format(
        "State(Wrist Angle: %.2f, Ele Height: %.2f)", wrist_angle.getRadians(), elevator_height);
  }
}
