package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.superstructure.anglegen.AngleGen;
import frc.robot.commands.superstructure.anglegen.SuperTraj;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.SuperVisualizer;
import frc.robot.subsystems.scoring.superstructure.Superstructure;

public class SuperToState extends Command {
    
    Superstructure superstructure;
    Timer timer;
    Supplier<SuperState> traj_states;
    SuperTraj trajectory;
    TrapezoidProfile tp = new TrapezoidProfile(new Constraints(20, 20));
    SuperState goal;
    
    public SuperToState(Superstructure superstruct, SuperState goalState) {
        addRequirements(superstruct);

        superstructure = superstruct;
        timer = new Timer();
        goal = goalState;
        traj_states = () -> goal;
    }

    public void initialize() {
        trajectory = AngleGen.getInstance().generateTrajectory(superstructure.getState(), goal);

        if (trajectory.getStates().isEmpty()) {
            traj_states = () -> superstructure.getState();
        } else {
            traj_states = () -> trajectory.sample(timer.get());
        }

        timer.start();
        System.out.println("Super go to state ...");

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
