package frc.robot.subsystems.scoring.superstructure;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;


public class SuperstructureConfig {

    public static final Joint wrist = new Joint(
        Kilograms.of(2.5401127).magnitude(),
        Meters.of(0.46478698).magnitude(), //from pivot to approximate center of algae
        KilogramSquareMeters.of(0.44).magnitude(),
        Meters.of(0.287).magnitude());



    //config.json has all of these at zero, these are values from the robot's origin -rn
    public static Pose3d origin = new Pose3d(0.0, 0.0, 0, new Rotation3d(0,0,0));

    public static Pose3d wristAxis = new Pose3d(0.30380, 0.03991, 0.16828 + 0.0127, new Rotation3d(0, 0, 0));
    public static Pose3d climberAxis = new Pose3d(-0.31670, -0.05011, 0.20003 + 0.0127, new Rotation3d());

    public record Joint(
        double mass,
        double length,
        double inertiaAbtCoM,
        double disFromPivot2CoG
    ){}
}
