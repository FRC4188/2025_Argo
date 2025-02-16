package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.robot.STATE;
import frc.robot.commands.autos.pathgen.PG_math;
import frc.robot.subsystems.scoring.anglegen.SuperTraj.SuperTrajState;
import frc.robot.util.FieldConstant.Reef;
import frc.robot.util.FieldConstant.Reef;


public class SuperState {
    //radians and meters
    private double wrist_angle;
    private double arm_angle;
    private double elevator_height;
    private boolean isCoral;

    
    public SuperState() {}

    //cartesian pose
    public SuperState(Translation2d endEffectorPos, boolean isCoral) {
        Translation3d guh = ArmKinematics.fromPose(endEffectorPos, isCoral);

        wrist_angle = guh.getX();
        arm_angle = guh.getY();
        elevator_height = guh.getZ();
    }

    public SuperState(Translation2d endEffectorPos) {
        Translation3d guh = ArmKinematics.fromPose(endEffectorPos, false);

        wrist_angle = guh.getX();
        arm_angle = guh.getY();
        elevator_height = guh.getZ();
    }

    public SuperState(Translation3d pose, boolean isCoral){
        wrist_angle = Units.degreesToRadians(pose.getY());
        arm_angle = Units.degreesToRadians(pose.getX());
        elevator_height = (pose.getZ());
        this.isCoral = isCoral;
    } 
    
    public SuperState(SuperTrajState trajectory_state) {
        wrist_angle = trajectory_state.poseMeters.getX();
        arm_angle = trajectory_state.poseMeters.getY();
        elevator_height = trajectory_state.poseMeters.getZ();
    }

    public SuperState(double ele, double arm, double wrist, boolean isCoral) {
        Translation2d pos = ArmKinematics.fromValues(Units.degreesToRadians(wrist), Units.degreesToRadians(arm), ele, isCoral);
        Translation3d guh = ArmKinematics.fromPose(pos, isCoral);
        wrist_angle = guh.getX();
        arm_angle = guh.getY();
        elevator_height = guh.getZ();
    }

    public double getWristAngle() {
        return wrist_angle;
    }  

    public double getArmAngle() {
        return arm_angle;
    }

    public double getHeight() {
        return elevator_height;
    }

    public double getWristOffset(){
        return isCoral? IntakeMode.CORAL.angleOffset : IntakeMode.ALGAE.angleOffset;
    }

    public Translation2d getCartesian() {
        return ArmKinematics.fromValues(wrist_angle, arm_angle, elevator_height, false);
    }

    public Translation2d getEndPos(){
        return ArmKinematics.forward(VecBuilder.fill(arm_angle, wrist_angle));
    }

    public boolean isCoral(){
        return isCoral;
    }

    public enum ElevatorPreset{
        MAX(0.0),
        MIN(0.0),
        L4(0.0),
        L3(0.0),
        L2(0.0),
        L1(0.0);

        final double height;
        ElevatorPreset(double height){
            this.height = height;
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

    public static enum SuperPreset{
        L4_CORAL(
            new SuperState(
                new Translation3d(Reef.L4_highest_h - Units.inchesToMeters(20), -25, -30), true)),
        L3_CORAL(
            new SuperState(
                new Translation3d(Reef.L3_highest_h - Units.inchesToMeters(20), -30, -25), true)),
        L3_ALGAE_REVERSE(
            new SuperState(
                Reef.L2_highest_h - Units.inchesToMeters(20), -30, -60, true)),
        L3_ALGAE(
            new SuperState(
                Reef.L3_highest_h - Units.inchesToMeters(20), 45, 30, false)),
        L2_CORAL(
            new SuperState(
                new Translation3d(Reef.L2_highest_h - Units.inchesToMeters(20), -30, -25), true)),
        L2_ALGAE(
            new SuperState(
                Reef.L2_highest_h - Units.inchesToMeters(20), 45, 30, false)),
        L2_ALGAE_REVERSE(
            new SuperState(
                Reef.L2_highest_h - Units.inchesToMeters(20), -30, -60, true)),
        PROCESSOR(
            new SuperState(
                0, 65, 15, false)),
        PROCESSOR_REVERSE(
            new SuperState(
                0, -65, -30, false)),
        SOURCE(
            new SuperState(
                Units.inchesToMeters(14), 45, 70, true)),
        SOURCE_REVERSE(
            new SuperState(
                Units.inchesToMeters(7), -30, 25, true)),
        // ALGAE_GROUND(
        //     new SuperState(
        //         new Translation3d(), false)),
        START(
            new SuperState(
                new Translation3d(0,0,0), false));

        private final SuperState state;

        private SuperPreset(SuperState state){
            this.state = state;
        }
        public SuperState getState(){
            return state;
        }
    }
}