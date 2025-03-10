package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;

public class SuperConstraints {

    public static Constraints SuperTrajConstraints = new Constraints(20, 20);

    public class WristConstraints{
        public final static double LOWEST_A = Units.degreesToRadians(0);
        public final static double HIGHEST_A = Units.degreesToRadians(90);
    }

    public class ElevatorConstraints{
        public final static double LOWEST_H = Units.inchesToMeters(9.13250);
        public final static double HIGHEST_H = Units.inchesToMeters(9.13250 + 70);
        public final static double RANGE = HIGHEST_H - LOWEST_H;
    }

    public static SuperState clamp(SuperState state) {
        double result_w = MathUtil.clamp(state.getWristAngle(), WristConstraints.LOWEST_A, WristConstraints.HIGHEST_A);
        double result_e = MathUtil.clamp(state.getEleHeight(), ElevatorConstraints.LOWEST_H, ElevatorConstraints.HIGHEST_H);

        return new SuperState(result_w, result_e);
    }

    //checks if state is conflicting with bumper or carriage
    public static boolean validState(SuperState state) {
        if (state.getEleHeight() < 0 || state.getEleHeight() > ElevatorConstraints.RANGE) return false;
        return state.getWristAngle() <  WristConstraints.LOWEST_A || state.getWristAngle()> WristConstraints.HIGHEST_A;
    }
}
