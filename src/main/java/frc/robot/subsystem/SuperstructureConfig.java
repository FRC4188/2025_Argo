package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import frc.robot.subsystem.ArmFF.Joint;

public class SuperstructureConfig {
    Joint arm = new Joint(
        Kilograms.of(0.4545).magnitude(), 
        Meters.of(0.3957).magnitude(), 
        KilogramSquareMeters.of(0.017675).magnitude(),
        Meters.of(0.1128).magnitude());

    //not from pivot but at the end of the arm's bar (VERY rough estimate)
    double wristRadiusCOG = (0.5011 + 0.1128) - 0.3957; 

    Joint wrist = new Joint(
        Kilograms.of(5.4545).magnitude(),
        Meters.of(0.46478698).magnitude(), //from pivot to approximate center of algae
        KilogramSquareMeters.of(1.363625).magnitude(),
        Meters.of(wristRadiusCOG).magnitude());

    //TODO: need starting, lowest, highest, point of origin of arm
    static final double LOWEST_H = Inches.of(8.990645).magnitude();
    static final double HIGHEST_H = Inches.of(8.990645 + 72.0).magnitude();
    
}
