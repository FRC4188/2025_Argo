package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.robot.STATE;
import frc.robot.commands.autos.pathgen.PG_math;
import frc.robot.subsystems.scoring.anglegen.SuperTraj.SuperTrajState;


public class SuperState {
    //radians and meters
    private double wrist_angle;
    private double arm_angle;
    private double elevator_height;
    
    public SuperState() {}

    //cartesian pose
    public SuperState(Translation2d endEffectorPos) {
        Translation3d guh = fromPose(endEffectorPos, false);

        wrist_angle = guh.getX();
        arm_angle = guh.getY();
        elevator_height = guh.getZ();
    }
    
    public SuperState(SuperTrajState trajectory_state) {
        wrist_angle = trajectory_state.poseMeters.getX();
        arm_angle = trajectory_state.poseMeters.getY();
        elevator_height = trajectory_state.poseMeters.getZ();
    }

    public SuperState(double wrist, double arm, double ele) {
        wrist_angle = wrist;
        arm_angle = arm;
        elevator_height = ele;
    }

    public static Translation3d fromPose(Translation2d t, boolean isCoral) {
        //get arm angle from optimal wrist angle
        double a_length = Units.inchesToMeters(16.1378264837);
        double w_length = Units.inchesToMeters(15.9504541222); //algae

        double x = MathUtil.clamp(t.getX(), -w_length-a_length, w_length + a_length);
        double z = MathUtil.clamp(t.getY(), 0, SuperConstraints.ElevatorConstraints.HIGHEST_H + a_length);

        ArrayList<Translation2d> possible_vals = new ArrayList<Translation2d>();

        for (double a = -Math.PI; a < Math.PI; a += 0.2) {
            double val1 = Math.PI - Math.asin((x - a_length * Math.sin(a))/w_length) - a;
            double val2 = Math.asin((x - a_length * Math.sin(a))/w_length) - a;

            if (!(Double.isNaN(val1) && Double.isNaN(val2))) {

                double w_val = 0;
                if (Double.isNaN(val1)) {
                    w_val = PG_math.modulate(Rotation2d.fromRadians(val2)).getRadians();
                } else if (Double.isNaN(val2)) {
                    w_val = PG_math.modulate(Rotation2d.fromRadians(val1)).getRadians();
                } else {
                    val1 = PG_math.modulate(Rotation2d.fromRadians(val1)).getRadians();
                    val2 = PG_math.modulate(Rotation2d.fromRadians(val2)).getRadians();
                    w_val = (Math.abs(val2) > Math.abs(val1))?val1:val2;
                }

                if (w_val <= 5/9.0 * Math.PI && w_val >= -5/9.0 * Math.PI) {

                    possible_vals.add(new Translation2d(w_val, a));
                } else {

                }
            } else {

            }
        }

        possible_vals.sort((Translation2d a1, Translation2d a2) -> 
            (Double.compare(
                Math.hypot(a1.getX(), a1.getY()), 
                Math.hypot(a2.getX(), a2.getY())))
        );

        if (possible_vals.isEmpty()) return new Translation3d();

        double result_w = possible_vals.get(0).getX();
        double result_a =  possible_vals.get(0).getY();
        double result_e = MathUtil.clamp(z - SuperConstraints.ElevatorConstraints.LOWEST_H
             - a_length * Math.cos(result_a)
              - w_length * Math.cos(result_w + result_a),
              0,
              SuperConstraints.ElevatorConstraints.RANGE);

        return new Translation3d(result_w, result_a, result_e);
    } 

    public static Translation2d fromValues(double w_angle, double a_angle, double e_height, boolean isCoral) {
        double a_length = Units.inchesToMeters(16.1378264837);
        double w_length = Units.inchesToMeters(15.9504541222); //algae

        double x = a_length * Math.sin(a_angle) + w_length * Math.sin(w_angle + a_angle);
        double z = e_height + SuperConstraints.ElevatorConstraints.LOWEST_H + a_length * Math.cos(a_angle) + w_length * Math.cos(w_angle + a_angle);

        return new Translation2d(x, z);
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

    public Translation2d getCartesian() {
        return fromValues(wrist_angle, arm_angle, elevator_height, false);
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
                new Translation2d(
                    0.0,
                    ElevatorPreset.L4.height)), true),
        L4_ALGAE_REVERSE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L4.height)), true),
        L4_ALGAE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L4.height)), false),
        L3_CORAL(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), true),
        L3_ALGAE_REVERSE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), true),
                    L3_ALGAE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), false),
        L2_CORAL(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), true),
        L2_ALGAE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), false),
        L2_ALGAE_REVERSE(
            new SuperState(
                new Translation2d(
                    0.0,
                    ElevatorPreset.L3.height)), true),
        PROCESSOR(
            new SuperState(
                new Translation2d()), false),
        SOURCE(
            new SuperState(
                new Translation2d()), true),
        ALGAE_GROUND(
            new SuperState(
                new Translation2d()), false),
        START(
            new SuperState(
                new Translation2d()), false);

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
}