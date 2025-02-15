package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;
import frc.robot.util.FieldConstant.Reef;

import static frc.robot.util.FieldConstant.*;

import java.util.ArrayList;

public class SuperState{
    private double wristAngle;
    private double armAngle;

    private double elevatorInch;
    private Translation3d endEffectorPos;
    private static boolean isCoral;

    public static enum SuperPreset{
        L4_CORAL(
            new SuperState(
                ElevatorPreset.L4.heightInch, -25, -30, true)),
        L3_CORAL(
            new SuperState(
                Units.metersToInches(Reef.L3_highest_h) - 20, -30, -25, true)),
        L3_ALGAE_REVERSE(
            new SuperState(
                Units.metersToInches(Reef.L2_highest_h) - 20, -30, -60, true)),
        L3_ALGAE(
            new SuperState(
                Units.metersToInches(Reef.L3_highest_h) - 20, 45, 30, false)),
        L2_CORAL(
            new SuperState(
                Units.metersToInches(Reef.L2_highest_h) - 20, -30, -25, true)),
        L2_ALGAE(
            new SuperState(
                Units.metersToInches(Reef.L2_highest_h) - 20, 45, 30, false)),
        L2_ALGAE_REVERSE(
            new SuperState(
                Units.metersToInches(Reef.L2_highest_h) - 20, -30, -60, true)),
        PROCESSOR(
            new SuperState(
                0, 65, 15, false)),
        PROCESSOR_REVERSE(
            new SuperState(
                0, -65, -30, false)),
        SOURCE(
            new SuperState(
                14, 45, 70, true)),
        SOURCE_REVERSE(
            new SuperState(
                7, -30, 25, true)),
        ALGAE_GROUND(
            new SuperState(
                new Translation3d(), false)),
        START(
            new SuperState(
                new Translation3d(), false));

        private final SuperState state;
    
        private SuperPreset(SuperState state){
            this.state = state;
        }
        public SuperState getState(){
            return state;
        }

        public double getWristOffset(){
            return isCoral ? IntakeMode.CORAL.angleOffset : IntakeMode.ALGAE.angleOffset;
        }
    }

    public SuperState(double elevatorInch, double armAngle, double wristAngle, boolean isCoral){
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.elevatorInch = elevatorInch;
        this.isCoral = isCoral;
        var temp = ArmKinematics.forward(VecBuilder.fill(armAngle,wristAngle));
        this.endEffectorPos = new Translation3d(temp.getX(), 0, temp.getY() + Units.inchesToMeters(elevatorInch));
    }

    public SuperState(Translation3d endEffectorPos, boolean isCoral){
        this.endEffectorPos = endEffectorPos;
        this.armAngle =  ArmKinematics.apply(
            new Pose2d(endEffectorPos.getX(), endEffectorPos.getZ(), new Rotation2d()))
            .get(0,0);
        this.wristAngle = ArmKinematics.apply(
            new Pose2d(endEffectorPos.getX(), endEffectorPos.getZ(), new Rotation2d()))
            .get(1,0);
        this.elevatorInch = Units.metersToInches(
            endEffectorPos.getZ() - ArmKinematics.forward(VecBuilder.fill(armAngle,wristAngle)).getY());
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


    //origin of endeffector pos = arm rest pos + elevator lowest height
    //up + down = pos.getz, y of kinematics
    public double getWristAngle(){
        return wristAngle;
    }
    public double getArmAngle(){
        return armAngle;
    }
    public double getHeightInch(){
        return elevatorInch;
    }
    public Translation3d getEndEffectorPos() {
        return endEffectorPos;
    }

    public enum ElevatorPreset{
        MAX(0.0),
        MIN(0.0),
        L4(Units.metersToInches(Reef.L4_highest_h) - 20),
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