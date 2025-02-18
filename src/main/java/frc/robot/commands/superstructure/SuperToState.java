package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.superstructure.anglegen.AngleGen;
import frc.robot.commands.superstructure.anglegen.SuperTraj;
import frc.robot.subsystems.scoring.Superstructure.SuperState;
import frc.robot.subsystems.scoring.Superstructure.SuperVisualizer;
import frc.robot.subsystems.scoring.Superstructure.SuperStructure;

public class SuperToState extends Command {
    
    SuperStructure superstructure;
    Timer timer;
    Supplier<SuperState> traj_states;
    SuperTraj trajectory;
    TrapezoidProfile tp = new TrapezoidProfile(new Constraints(10, 10));
    SuperState goal;
    
    public SuperToState(SuperStructure superstruct, SuperState goalState) {
        superstructure = superstruct;
        timer = new Timer();
        goal = goalState;
        traj_states = () -> goal;
    }

    public void initialize() {
        trajectory = AngleGen.getInstance().generateTrajectory(superstructure.getState(), goal, tp);

        System.out.println(trajectory);

        if (trajectory.getStates().isEmpty()) {
            traj_states = () -> superstructure.getState();
        } else {
            traj_states = () -> trajectory.sample(timer.get());
        }

        timer.start();
    }

    public void execute() {
        superstructure.setTarget(traj_states.get());
    }

    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public void end(boolean interrupted) {
        superstructure.setTarget(goal);
    }
}