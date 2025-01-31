package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;


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


    //TODO: need starting, lowest, highest, point of origin of arm
    static final double LOWEST_H = Inches.of(8.990645).magnitude();
    static final double HIGHEST_H = Inches.of(8.990645 + 72.0).magnitude();

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
