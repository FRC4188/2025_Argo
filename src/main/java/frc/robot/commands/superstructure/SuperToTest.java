package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.SuperState;
import frc.robot.subsystems.scoring.SuperVisualizer;
import frc.robot.subsystems.scoring.anglegen.AngleGen;
import frc.robot.subsystems.scoring.anglegen.SuperTraj;

public class SuperToTest extends Command {
    
    SuperVisualizer testVisualizer;
    Timer timer;
    Supplier<SuperState> traj_states;
    SuperTraj trajectory;
    TrapezoidProfile tp;
    SuperState start;
    SuperState goal;
    
    public SuperToTest(SuperVisualizer sv, SuperState start, SuperState goal, TrapezoidProfile tp) {
        testVisualizer = sv;
        this.tp = tp;
        timer = new Timer();
        this.goal = goal;
        traj_states = () -> goal;

        this.start = start;
    }

    public void initialize() {
        trajectory = AngleGen.getInstance().generateTrajectory(start, goal, tp);

        System.out.println(trajectory);

        if (trajectory.getStates().isEmpty()) {
            traj_states = () -> start;
        } else {
            traj_states = () -> trajectory.sample(timer.get());
        }

        timer.start();
    }

    public void execute() {
        System.out.println(timer.get() + ": " + traj_states.get());
        testVisualizer.update(traj_states.get());
    }

    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public void end(boolean interrupted) {
        testVisualizer.update(goal);
    }


}
