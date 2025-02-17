package frc.robot.subsystems.scoring;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;


public class SuperState {
    //pivot constants; to be moved somewhere else 
    public static double arm_length = Units.inchesToMeters(16.1378264837);
    public static Pose2d goal_algae_offset = new Pose2d(0.033002, 0.40380, new Rotation2d());
    public static Pose2d goal_coral_offset = new Pose2d(0.276283, 0.30817, Rotation2d.fromRadians(Math.atan2(-0.30817, 0.276283)));

    //radians and meters
    private double wrist_angle = 0;
    private double arm_angle = 0;
    private double elevator_height = 0;
    
    public SuperState() {}

    //cartesian pose
    public SuperState(Translation2d endEffectorPos, double optimal_angle, boolean isCoral) {
        SuperState calculated = fromPose(endEffectorPos, optimal_angle, isCoral);

        wrist_angle = calculated.getWristAngle();
        arm_angle = calculated.getArmAngle();
        elevator_height = calculated.getEleHeight();
    }

    public SuperState(double wrist, double arm, double elevator) {
        wrist_angle = wrist;
        arm_angle = arm;
        elevator_height = elevator;
    }

    public static SuperState fromPose(Translation2d t, double optimal_score_angle, boolean isCoral) {
        double wrist_length = (isCoral)?goal_coral_offset.getTranslation().getNorm():goal_algae_offset.getTranslation().getNorm();
        double angle_offset = (isCoral)?goal_coral_offset.getRotation().getRadians():goal_algae_offset.getRotation().getRadians();

        double x = MathUtil.clamp(t.getX(), -wrist_length - arm_length,  wrist_length + arm_length);
        double z = MathUtil.clamp(t.getY(), 0, SuperConstraints.ElevatorConstraints.HIGHEST_H + arm_length);

        ArrayList<Translation2d> possible_vals = new ArrayList<Translation2d>();

        for (double a = SuperConstraints.ArmConstraints.LOWEST_A; a < SuperConstraints.ArmConstraints.HIGHEST_A; a += 0.3) {

            double val1 = Math.PI - Math.asin((x - arm_length * Math.sin(a))/wrist_length) - a - angle_offset;
            double val2 = Math.asin((x - arm_length * Math.sin(a))/wrist_length) - a - angle_offset;
            
                if (!Double.isNaN(val1) ) {
                    val1 = PG_math.modulate(Rotation2d.fromRadians(val1)).getRadians();
                }

                if (!Double.isNaN(val2)) {
                    val2 = PG_math.modulate(Rotation2d.fromRadians(val2)).getRadians();
                }

                if (val1 <=SuperConstraints.WristConstraints.HIGHEST_A && val1 >= SuperConstraints.WristConstraints.LOWEST_A) {


                    possible_vals.add(new Translation2d(val1, a));
                }

                if (val2 <=SuperConstraints.WristConstraints.HIGHEST_A && val2 >= SuperConstraints.WristConstraints.LOWEST_A) {

                    possible_vals.add(new Translation2d(val2, a));
                }
    
        }

        possible_vals.sort((Translation2d a1, Translation2d a2) -> 
            (Double.compare(
                Math.abs(optimal_score_angle - (a1.getX() + a1.getY())), 
                Math.abs(optimal_score_angle - (a2.getX() + a2.getY()))))
        );


        for (Translation2d angles : possible_vals) {
            double result_w = angles.getX();
            double result_a = angles.getY();
            double result_e = z - SuperConstraints.ElevatorConstraints.LOWEST_H - arm_length * Math.cos(result_a)- wrist_length * Math.cos(result_w + result_a + angle_offset);

            SuperState result = new SuperState(result_w, result_a, result_e);
            
            if (SuperConstraints.validState(result)) {
                 return result;
             }
        }

        return new SuperState();
    } 

    public static Translation2d fromValues(double w_angle, double a_angle, double e_height, boolean isCoral) {
        double wrist_length = (isCoral)?goal_coral_offset.getTranslation().getNorm():goal_algae_offset.getTranslation().getNorm();
        double angle_offset = (isCoral)?goal_coral_offset.getRotation().getRadians():goal_algae_offset.getRotation().getRadians();

        double x = arm_length * Math.sin(a_angle) + wrist_length  * Math.sin(w_angle + a_angle  - angle_offset);
        double z = e_height + SuperConstraints.ElevatorConstraints.LOWEST_H + arm_length * Math.cos(a_angle) + wrist_length * Math.cos(w_angle + a_angle - angle_offset);

        return new Translation2d(x, z);
    }

    public double getWristAngle() {
        return wrist_angle;
    }  

    public double getArmAngle() {
        return arm_angle;
    }

    public double getScoreAngle(boolean isCoral) {
        return arm_angle + wrist_angle + ((isCoral)?goal_coral_offset.getRotation().getRadians():goal_algae_offset.getRotation().getRadians());
    }

    public double getGlobalAngle() {
        return arm_angle + wrist_angle;
    }

    public double getEleHeight() {
        return elevator_height;
    }

    public static double getWristOffset(){
        return goal_coral_offset.getRotation().getRadians();
    }

    public Translation2d getCartesian(boolean isCoral) {
        return fromValues(wrist_angle, arm_angle, elevator_height, isCoral);
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
        /* 
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
        */
        START(
            new SuperState(0, 0, 0));
        
        private final SuperState state;

        private SuperPreset(SuperState state){
            this.state = state;
        }
        public SuperState getState(){
            return state;
        }
    }

    public String toString() {
        return String.format("State(Wrist Angle: %.2f, Arm Angle: %.2f, Ele Height: %.2f)", wrist_angle, arm_angle, elevator_height);
    }
}