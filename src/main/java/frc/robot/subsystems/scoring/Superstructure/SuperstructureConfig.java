
package frc.robot.subsystems.scoring.superstructure;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;


public class SuperstructureConfig {
    public static final Joint arm = new Joint(
        Kilograms.of(0.4545).magnitude(), 
        Meters.of(0.3957).magnitude(), 
        KilogramSquareMeters.of(0.017675).magnitude(),
        Meters.of(0.1128).magnitude());

    //not from pivot but at the end of the arm's bar (VERY rough estimate)
    private static double wristRadiusCOG = (0.5011 + 0.1128) - 0.3957; 

    public static final Joint wrist = new Joint(
        Kilograms.of(5.4545).magnitude(),
        Meters.of(0.46478698).magnitude(), //from pivot to approximate center of algae
        KilogramSquareMeters.of(1.363625).magnitude(),
        Meters.of(wristRadiusCOG).magnitude());



    //config.json has all of these at zero, these are values from the robot's origin -rn
    public static Pose3d origin = new Pose3d(0.0, 0.0, 0, new Rotation3d(0,0,0));
    public static Pose3d carriageOrigin = new Pose3d(0.000172, -0.00711, 0.23197, new Rotation3d(Units.degreesToRadians(180),0, 0));
    public static Pose3d armOrigin = new Pose3d(-0.004077, 0.000171, 0.231965, new Rotation3d(Units.degreesToRadians(180), 0, 0));
    public static Pose3d wristOrigin = new Pose3d(-0.004077, -0.049069, 0.644627, new Rotation3d(Units.degreesToRadians(180),0, 0));

    public record Joint(
        double mass,
        double length,
        double inertiaAbtCoM,
        double disFromPivot2CoG
    ){}

    public enum IntakeMode{
        ALGAE(0.0),
        CORAL(0.0),
        ALGAE_FEEDING(0.0); //need testing to see if possible to descore w just 1 side of the algae wheels + if it gets stuck

        double angleOffset;
        IntakeMode(double angleOffset){
            this.angleOffset = angleOffset;
        }
    }
}
