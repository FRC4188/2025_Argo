package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.SuperState;
import frc.robot.subsystems.scoring.SuperVisualizer;
import frc.robot.subsystems.scoring.Superstructure;
import frc.robot.subsystems.scoring.anglegen.AngleGen;
import frc.robot.subsystems.scoring.anglegen.SuperTraj;
import frc.robot.subsystems.scoring.anglegen.SuperTraj.SuperTrajState;


public class SuperToTest extends Command {
    
    SuperVisualizer testVisualizer;
    Timer timer;
    Supplier<SuperState> traj_states;
    SuperTraj trajectory;
    TrajectoryConfig config;
    SuperState start;
    SuperState goal;
    
    public SuperToTest(SuperVisualizer sv, SuperState goal, TrajectoryConfig config) {
        testVisualizer = sv;
        this.config = config;
        timer = new Timer();
        this.goal = goal;
        traj_states = () -> goal;

        start = new SuperState(new Translation2d(0, 0));
    }

    public void initialize() {
        trajectory = AngleGen.getInstance().generateTrajectory(start, goal, config);

        if (trajectory.getStates().isEmpty()) {
            traj_states = () -> start;
        } else {
            traj_states = () -> new SuperState(trajectory.sample(timer.get()));
        }

        timer.start();
    }

    public void execute() {
        testVisualizer.update(traj_states.get().getHeight(), traj_states.get().getArmAngle(), traj_states.get().getWristAngle());
    }

    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public void end(boolean interrupted) {
        testVisualizer.update(goal.getHeight(), goal.getArmAngle(), goal.getWristAngle());
    }


}
