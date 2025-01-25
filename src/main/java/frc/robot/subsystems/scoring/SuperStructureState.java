package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;

public class SuperStructureState {
    public record SuperState(Pose3d pose){

        public double getWristAngle(){
            return 0; //TODO: apply kinematics
        }
        public double getArmAngle(){
            return 0; //TODO: apply kinematics
        }
        public double getHeightInch(){
            return Inches.convertFrom(pose.getZ(), Meters);
        }

    }

    public enum ElevatorPreset{
        MAX(0.0),
        MIN(0.0),
        L4(0.0),
        L3(0.0),
        L2(0.0),
        L1(0.0);

        final double heightInch;
        ElevatorPreset(double heightInch){
            this.heightInch = heightInch;
        }
    }
}
