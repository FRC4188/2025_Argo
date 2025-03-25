package frc.robot.commands.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;
import frc.robot.subsystems.scoring.superstructure.SuperState;
import frc.robot.subsystems.scoring.superstructure.Superstructure;
import frc.robot.subsystems.scoring.superstructure.SuperState.SuperPreset;
import frc.robot.subsystems.scoring.wrist.Wrist;

public class SuperToState extends SequentialCommandGroup {

    public SuperToState(Superstructure superstruct, double safeangle, SuperState state) {
        addRequirements(superstruct);

        addCommands(
            new WristToState(superstruct, safeangle),
            new EleToState(superstruct, state.getEleHeight()),
            new WristToState(superstruct, state.getWristAngle())
        );
    }

    public SuperToState(Superstructure superstruct, double wrist_angle) {
        addRequirements(superstruct);
        addCommands(
            new WristToState(superstruct, wrist_angle)
        );
    }

    private static class EleToState extends Command {
        private Superstructure superstructure;
        private double height;

        public EleToState(Superstructure superstructure, double height) {
            this.superstructure = superstructure;
            this.height = MathUtil.clamp(height,0,  SuperConstraints.ElevatorConstraints.RANGE);
        }

        public void initialize(){
        
            superstructure.setEle(height);
        }

        public boolean isFinished() {
            return Constants.robot.currMode == Mode.SIM? true: superstructure.eleAtTarget();
        }
    }

    private static class WristToState extends Command {
        private Superstructure superstructure;
        private double angle;

        public WristToState(Superstructure superstructure, double angle) {
            this.superstructure = superstructure;
            this.angle = MathUtil.clamp(angle,SuperConstraints.WristConstraints.LOWEST_A, SuperConstraints.WristConstraints.HIGHEST_A);
        }

        public void initialize(){
            superstructure.setWrist(angle);
        }

        public boolean isFinished() {
            return Constants.robot.currMode == Mode.SIM? true: superstructure.wristAtTarget();
        }
    }
    
}
