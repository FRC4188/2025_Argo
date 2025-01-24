package frc.robot.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import frc.robot.subsystem.ArmFF.Joint;

public class ArmConfig {
    Joint arm = new Joint(
        Pounds.of(0.299).magnitude(), 
        Meters.of(0.01604).magnitude(), 
        0.0,
        0.0);

    // Joint wrist = new Joint(
    //     , 0, 0, 0)
}
