package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record SuperState(Translation3d endEffectorPos){

    public static enum SuperPreset{
        L4_CORAL(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L4.heightInch, Inches))), true),
        L4_CORAL_REVERSE(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L4.heightInch, Inches))), true),
        L4_ALGAE(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L4.heightInch, Inches))), false),
        L3_CORAL(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L3.heightInch, Inches))), true),
        L3_ALGAE(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L3.heightInch, Inches))), false),
        L2_CORAL(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L2.heightInch, Inches))), true),
        L2_ALGAE(
            new SuperState(
                new Translation3d(
                    0.0,
                    0.0,
                    Meters.convertFrom(ElevatorPreset.L2.heightInch, Inches))), false),
        PROCESSOR(
            new SuperState(
                new Translation3d()), false),
        SOURCE(
            new SuperState(
                new Translation3d()), true),
        ALGAE_GROUND(
            new SuperState(
                new Translation3d()), false);

        private final SuperState state;
        private final boolean isCoral;
    
        private SuperPreset(SuperState state, boolean isCoral){
            this.state = state;
            this.isCoral = isCoral;
        }
        public SuperState getState(){
            return state;
        }

        public double getWristOffset(){
            return isCoral? IntakeMode.CORAL.angleOffset : IntakeMode.ALGAE.angleOffset;
        }
    }


    //origin of endeffector pos = arm rest pos + elevator lowest height
    //up + down = pos.getz, y of kinematics
    public double getWristAngle(){
        return ArmKinematics.apply(
            new Pose2d(endEffectorPos.getX(), endEffectorPos.getZ(), new Rotation2d()))
            .get(1,0);
    }
    public double getArmAngle(){
        return ArmKinematics.apply(
            new Pose2d(endEffectorPos.getX(), endEffectorPos.getZ(), new Rotation2d()))
            .get(0,0);
    }
    public double getHeightInch(){
        return Inches.convertFrom(endEffectorPos.getZ(), Meters);
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