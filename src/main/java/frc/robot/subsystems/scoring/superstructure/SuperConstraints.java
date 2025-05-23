package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;

public class SuperConstraints {
    
    public class WristConstraints{
        public final static double LOWEST_A = 0;
        public final static double HIGHEST_A = Units.degreesToRadians(80);
    }

    public class ElevatorConstraints{
        public final static double LOWEST_H = Units.inchesToMeters(9.13250);
        public final static double HIGHEST_H = Units.inchesToMeters(9.13250 + 72);
        public final static double RANGE = HIGHEST_H - LOWEST_H;
    }

    public static SuperState clamp(SuperState state) {
        double result_w = MathUtil.clamp(state.getWristAngle(), WristConstraints.LOWEST_A, WristConstraints.HIGHEST_A);
        double result_e = MathUtil.clamp(state.getEleHeight(), 0, ElevatorConstraints.RANGE);

        return new SuperState(result_w, result_e);
    }
}