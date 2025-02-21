package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;

public class SuperConstraints {

    public static Constraints SuperTrajConstraints = new Constraints(20, 20);

    public class ArmConstraints {
        public final static double LOWEST_A = Units.degreesToRadians(-180);
        public final static double HIGHEST_A = Units.degreesToRadians(180);
    }

    public class WristConstraints{
        public final static double LOWEST_A = Units.degreesToRadians(-100);
        public final static double HIGHEST_A = Units.degreesToRadians(100);
    }

    public class ElevatorConstraints{
        public final static double LOWEST_H = Units.inchesToMeters(9.13250);
        public final static double HIGHEST_H = Units.inchesToMeters(9.13250 + 70);
        public final static double RANGE = HIGHEST_H - LOWEST_H;
    }

    public static SuperState clamp(SuperState state) {
        double result_a = MathUtil.clamp(state.getArmAngle(), ArmConstraints.LOWEST_A, ArmConstraints.HIGHEST_A);
        double result_w = MathUtil.clamp(state.getWristAngle(), WristConstraints.LOWEST_A, WristConstraints.HIGHEST_A);
        double result_e = MathUtil.clamp(state.getEleHeight(), ElevatorConstraints.LOWEST_H, ElevatorConstraints.HIGHEST_H);

        return new SuperState(result_w, result_a, result_e);
    }

    //checks if state is conflicting with bumper or carriage
    public static boolean validState(SuperState state) {
        if (state.getEleHeight() < 0 || state.getEleHeight() > ElevatorConstraints.RANGE) return false;
        if (state.getArmAngle() <  ArmConstraints.LOWEST_A || state.getArmAngle() >  ArmConstraints.HIGHEST_A) return false;
        if (state.getWristAngle() <  WristConstraints.LOWEST_A || state.getWristAngle()> WristConstraints.HIGHEST_A) return false;

        //local inches, do not mess with avlues
        Translation2d b_tl = new Translation2d(-5.195177165, 18.24324);
        Translation2d b_tr = new Translation2d(-5.195177165, -18.24324);
        Translation2d b_br = new Translation2d(-9.132677165, -18.24324);
        Translation2d b_bl = new Translation2d(-9.132677165, 18.24324);

        double ground = -9.132677165;

        Translation2d c_tr = new Translation2d(4.5, -6.5);
        Translation2d c_tl = new Translation2d(4.5, 6.5);
        Translation2d c_br = new Translation2d(-4.5, -6.5);
        Translation2d c_bl = new Translation2d(-4.5, 6.5);

        Translation2d worigin = new Translation2d(Units.metersToInches(state.getEleHeight()), 0).plus( 
            new Translation2d(16.1378264837, 0).rotateBy(Rotation2d.fromRadians(state.getArmAngle())));

        
        Translation2d w_tr = worigin.plus(new Translation2d(13.1989, -9.443734).rotateBy(Rotation2d.fromRadians(state.getGlobalAngle())));
        Translation2d w_br = worigin.plus(new Translation2d(2.619168, -9.443734).rotateBy(Rotation2d.fromRadians(state.getGlobalAngle())));
        Translation2d w_tl = worigin.plus(new Translation2d(13.1989, 9.443734).rotateBy(Rotation2d.fromRadians(state.getGlobalAngle())));
        Translation2d w_bl = worigin.plus(new Translation2d(2.619168, 9.443734).rotateBy(Rotation2d.fromRadians(state.getGlobalAngle())));

        Translation2d ec_tr = c_tr.plus(new Translation2d(Units.metersToInches(state.getEleHeight()), 0));
        Translation2d ec_tl = c_tl.plus(new Translation2d(Units.metersToInches(state.getEleHeight()), 0));
        Translation2d ec_br = c_br.plus(new Translation2d(Units.metersToInches(state.getEleHeight()), 0));
        Translation2d ec_bl = c_bl.plus(new Translation2d(Units.metersToInches(state.getEleHeight()), 0));
        
        return !(
            PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_tr, ec_tl) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_br, ec_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_bl, ec_br) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_tl, ec_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_tr, ec_tl) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_br, ec_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_bl, ec_br) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_tl, ec_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, b_tr, b_tl) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, b_br, b_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tr, w_br, b_bl, b_tl) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_tr, b_tl) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_br, b_tr) <= 0.5 ||
            PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_bl, b_tl) <= 0.5 ||
            w_tr.getX() < ground ||
            w_tl.getX() < ground ||
            w_br.getX() < ground ||
            w_bl.getX() < ground
            );
    }
}
