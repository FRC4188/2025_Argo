package frc.robot.subsystems.scoring.anglegen;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.scoring.anglegen.SuperTraj.SuperTrajState;

public class SuperTraj {
    private final double m_totalTimeSeconds;
    private final List<SuperTrajState> m_states;

    public SuperTraj() {
        m_states = new ArrayList<>();
        m_totalTimeSeconds = 0.0;
    }

    public SuperTraj(final List<SuperTrajState> states) {
        m_states = states;

        if (m_states.isEmpty()) {
            throw new IllegalArgumentException("Trajectory manually created with no states.");
        }

        m_totalTimeSeconds = m_states.get(m_states.size() - 1).timeSeconds;
    }
   
    public static SuperTraj generateSuperTraj(List<Translation3d> waypoints, double avg_dis, TrajectoryConfig config) {
        double max_acc = config.getMaxAcceleration();
        double max_vel = config.getMaxVelocity();
        double distance = (waypoints.size() - 1) * avg_dis;

        double time_acc = (distance >= max_vel * max_vel / max_acc)?max_vel/max_acc : Math.sqrt(distance / max_acc);
        double time_vel = Math.max(0, (distance - max_vel * max_vel / max_acc) / max_vel);

        double d1 = 0.5 * max_acc * time_acc * time_acc;
        double d2 = max_vel * (0.5 * time_acc + time_vel);

        List<SuperTrajState> states = new ArrayList<SuperTrajState>();

        for (int i = 0; i < waypoints.size(); i++) {
            double pos = i * avg_dis;

            double time = 
                (pos < d1)?Math.sqrt(2 * pos / max_acc):
                (pos < d2)?pos / max_vel + time_acc / 2: 
                -Math.sqrt(-2 / max_vel * (pos - distance)) + 2 * time_acc + time_vel;

            double vel = 
                (time < time_acc)?max_acc * time:
                (time < time_acc + time_vel)?max_vel:
                -max_acc * (time - 2 * time_acc - time_vel);

            double acc = 
                (time < time_acc)?max_acc:
                (time < time_acc + time_vel)?0:
                -max_acc;

            
            states.add(new SuperTrajState(time, vel, acc, waypoints.get(i)));

            System.out.println(states.get(states.size() - 1));
        }

        return new SuperTraj(states);
    }

    private static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    private static Translation3d lerp(Translation3d startValue, Translation3d endValue, double t) {
        return startValue.plus((endValue.minus(startValue)).times(t));
    }


    public Translation3d getInitialPose() {
        return sample(0).poseMeters;
    }


    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    public List<SuperTrajState> getStates() {
        return m_states;
    }

    public SuperTrajState sample(double timeSeconds) {
        if (m_states.isEmpty()) {
            throw new IllegalStateException("Trajectory cannot be sampled if it has no states.");
        }

        if (timeSeconds <= m_states.get(0).timeSeconds) {
            return m_states.get(0);
        }
        if (timeSeconds >= m_totalTimeSeconds) {
            return m_states.get(m_states.size() - 1);
        }

        int low = 1;
        int high = m_states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (m_states.get(mid).timeSeconds < timeSeconds) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        final SuperTrajState sample = m_states.get(low);
        final SuperTrajState prevSample = m_states.get(low - 1);

        if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9) {
            return sample;
        }

        return prevSample.interpolate(
            sample,
            (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
    }
    
    public static class SuperTrajState {
        public double timeSeconds;
        public double velocityMetersPerSecond;
        public double accelerationMetersPerSecondSq;
        public Translation3d poseMeters;

        public SuperTrajState() {
            poseMeters = Translation3d.kZero;
        }

        public SuperTrajState(
            double timeSeconds,
            double velocityMetersPerSecond,
            double accelerationMetersPerSecondSq,
            Translation3d poseMeters) {

            this.timeSeconds = timeSeconds;
            this.velocityMetersPerSecond = velocityMetersPerSecond;
            this.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq;
            this.poseMeters = poseMeters;
        }

        SuperTrajState interpolate(SuperTrajState endValue, double i) {
            final double newT = lerp(timeSeconds, endValue.timeSeconds, i);
            final double deltaT = newT - timeSeconds;

            if (deltaT < 0) {
                return endValue.interpolate(this, 1 - i);
            }

            final boolean reversing =
                velocityMetersPerSecond < 0
                || Math.abs(velocityMetersPerSecond) < 1E-9 && accelerationMetersPerSecondSq < 0;

            final double newV = velocityMetersPerSecond + (accelerationMetersPerSecondSq * deltaT);

            final double newS =
                (velocityMetersPerSecond * deltaT
                  + 0.5 * accelerationMetersPerSecondSq * Math.pow(deltaT, 2))
                * (reversing ? -1.0 : 1.0);

            final double interpolationFrac =
                newS / endValue.poseMeters.getDistance(poseMeters);

            return new SuperTrajState(
                newT,
                newV,
                accelerationMetersPerSecondSq,
                lerp(poseMeters, endValue.poseMeters, interpolationFrac));
        }
        public String toString() {
            return String.format(
                "State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Pose: %s)",
                timeSeconds,
                velocityMetersPerSecond,
                accelerationMetersPerSecondSq,
                poseMeters);
        }
    }

    public String toString() {
        String stateList = m_states.stream().map(SuperTrajState::toString).collect(Collectors.joining(", \n"));
        return String.format("Trajectory - Seconds: %.2f, States:\n%s", m_totalTimeSeconds, stateList);
    }
}
