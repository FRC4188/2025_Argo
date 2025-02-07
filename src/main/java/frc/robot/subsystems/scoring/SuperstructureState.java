package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;

public class SuperstructureState {
    public record SuperState(Pose3d endEffectorPos){

        public double getWristAngle(){
            
            return 0; //TODO: apply kinematics
        }
        public double getArmAngle(){
            return 0; //TODO: apply kinematics
        }
        public double getHeightInch(){
            return Inches.convertFrom(endEffectorPos.getZ(), Meters);
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
