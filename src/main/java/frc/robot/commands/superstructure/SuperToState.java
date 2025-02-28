package frc.robot.commands.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;

public class SuperToState extends SequentialCommandGroup {
    Superstructure superstructure;

    public SuperToState(Superstructure superstructure, SuperState goal) {
        addCommands(
            new JointToState(superstructure, SuperPreset.START.getState()).onlyIf(() -> Intake.intakeState != Intake.Mode.ALGAE),
            new EleToState(superstructure, goal.getEleHeight()),
            new JointToState(superstructure, goal)
        );
    }

    private static class EleToState extends Command {
        Superstructure superstruct;
        double target = 0;

        public EleToState(Superstructure superstructure, double target) {
            this.superstruct = superstructure;
            this.target = target;
        }

        public void initialize() {
            superstruct.setEle(target);
        }

        public boolean isFinished() {
            return superstruct.eleAtTarget();
        }
    }

    private static class JointToState extends Command {
        Superstructure superstructure;
        Timer timer;
        Supplier<Translation2d> traj_states;
        Trajectory trajectory;
        TrajectoryConfig config = new TrajectoryConfig(10, 10);
        SuperState goal;

        public JointToState(Superstructure superstruct, SuperState goal) {
            this.goal = goal;
            this.superstructure = superstruct;
            timer = new Timer();
        }

        @Override
        public void initialize() {
            trajectory = AngleGen.getInstance().generateTrajectory(
                superstructure.getState(), 
                goal,
                config);

            if (trajectory.getStates().isEmpty()) {
                traj_states = () -> new Translation2d(goal.getWristAngle(), goal.getArmAngle());
            } else {
                traj_states = () -> trajectory.sample(timer.get()).poseMeters.getTranslation();
            }

            timer.start();
        }

        @Override
        public void execute() {
            Translation2d cur = traj_states.get();
            superstructure.setArm(cur.getY());
            superstructure.setWrist(cur.getX());
        }

        @Override
        public boolean isFinished() {
            return timer.hasElapsed(trajectory.getTotalTimeSeconds());
        }

        public void end(boolean interrupted) {
            superstructure.setArm(goal.getArmAngle());
            superstructure.setWrist(goal.getWristAngle());
        }
    }
}
