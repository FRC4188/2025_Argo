package frc.robot.commands.superstructure.anglegen;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.scoring.Superstructure.SuperState;

public class SuperTraj {
    private final double m_totalTimeSeconds;
    private final List<SuperState> m_states;

    private TrapezoidProfile tp;
    private double distance;

    public SuperTraj() {
        m_states = new ArrayList<>();
        m_totalTimeSeconds = 0.0;
    }

    public SuperTraj(final List<SuperState> states, double distance, TrapezoidProfile tp) {
        m_states = states;
        this.tp = tp;
        this.distance = distance;
        if (m_states.isEmpty()) {
            throw new IllegalArgumentException("Trajectory manually created with no states.");
        }
        tp.calculate(0, new State(0, 0), new State(distance, 0));
        m_totalTimeSeconds = tp.totalTime();
    }

    public static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    public static SuperState lerp(SuperState startValue, SuperState endValue, double t) {
        return new SuperState(
            startValue.getWristAngle() + (endValue.getWristAngle() - startValue.getWristAngle()) * t,
            startValue.getArmAngle() + (endValue.getArmAngle() - startValue.getArmAngle()) * t,
            startValue.getEleHeight() + (endValue.getEleHeight() - startValue.getEleHeight()) * t
            );
    }


    public SuperState getInitialPose() {
        return sample(0);
    }


    public double getTotalTimeSeconds() {
        return m_totalTimeSeconds;
    }

    public List<SuperState> getStates() {
        return m_states;
    }

    public SuperState sample(double timeSeconds) {
        if (m_states.isEmpty()) {
            throw new IllegalStateException("Trajectory cannot be sampled if it has no states.");
        }

        if (timeSeconds <= 0) {
            return m_states.get(0);
        }
        if (timeSeconds >= m_totalTimeSeconds) {
            return m_states.get(m_states.size() - 1);
        }
        State lerpD = tp.calculate(timeSeconds, new State(0, 0), new State(distance, 0));
        if (lerpD.position >= distance) return m_states.get(m_states.size() - 1);

        return lerp(m_states.get((int)lerpD.position), m_states.get((int)lerpD.position + 1), lerpD.position - Math.floor(lerpD.position));
    }

    public String toString() {
        String stateList = m_states.stream().map(SuperState::toString).collect(Collectors.joining(", \n"));
        return String.format("Trajectory - Seconds: %.2f, States:\n%s", m_totalTimeSeconds, stateList);
    }
}