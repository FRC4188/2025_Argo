
package frc.robot.subsystems.scoring.Superstructure;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.arm.ArmIOReal;
import frc.robot.subsystems.scoring.arm.ArmIOSim;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.elevator.ElevatorIOReal;
import frc.robot.subsystems.scoring.elevator.ElevatorIOSim;
import frc.robot.subsystems.scoring.ArmFF;
import frc.robot.subsystems.scoring.Superstructure.SuperState.*;
import frc.robot.subsystems.scoring.wrist.Wrist;
import frc.robot.subsystems.scoring.wrist.WristIOReal;
import frc.robot.subsystems.scoring.wrist.WristIOSim;

public class SuperStructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;

    private SuperVisualizer sim;

    ArmFF ff;

    private final Constraints constraints = new Constraints(Units.degreesToRadians(960.0), Units.degreesToRadians(720.0));

    private ProfiledPIDController armPID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);

    private ProfiledPIDController wristPID = 
        new ProfiledPIDController(
            2, 0, 3,
            constraints);


    private ProfiledPIDController elePID = 
        new ProfiledPIDController(
            1, 0.0, 0.0, 
            constraints);

    private ElevatorFeedforward eleff =
        new ElevatorFeedforward(
            0.1, 0, 0);

    private ArmFeedforward wristff = 
        new ArmFeedforward(0.1, 0, 0);
        
    private SuperState target;
    private SuperState current;
    
    public SuperStructure(Mode mode){
        switch (mode) {
            case REAL:
                this.arm = new Arm(new ArmIOReal());
                this.wrist = new Wrist(new WristIOReal());
                this.elevator = new Elevator(new ElevatorIOReal());
                break;
            case SIM: 
                this.arm = new Arm(new ArmIOSim());
                this.wrist = new Wrist(new WristIOSim());
                this.elevator = new Elevator(new ElevatorIOSim());
                break;
            default: 
                this.arm = new Arm(new ArmIOSim());
                this.wrist = new Wrist(new WristIOSim());
                this.elevator = new Elevator(new ElevatorIOSim());
        }
        sim = new SuperVisualizer("Superstructure");

        target = SuperPreset.START.getState();

        ff = new ArmFF();

        this.current = new SuperState(
            wrist.getAngle(),
            arm.getAngle(),
            elevator.getHeight());
    }

    public void setgoal(SuperState goal){
        target = goal;

    }


    @Override
    public void periodic(){

        current = new SuperState(
            wrist.getAngle(),
            arm.getAngle(),
            elevator.getHeight());

        sim.update(current);

        var ffVolt = ff.calculate(
            VecBuilder.fill(target.getArmAngle(), target.getWristAngle())
        );

        arm.runVolt(
            armPID.calculate(arm.getAngle(), target.getArmAngle())
            //+ ffVolt.get(0, 0)
            //^- until singlejointedarmsim gets workin, using for other stuff like autos
        );

        elevator.runVolts(
            elePID.calculate(Units.metersToInches(elevator.getHeight()), Units.metersToInches(target.getEleHeight())) + eleff.calculate(20)
        );

        wrist.runVolts(
            wristPID.calculate(wrist.getAngle(), target.getWristAngle())       
            + wristff.calculate(target.getGlobalAngle() + Math.PI/2, 0)
        );
        
        wrist.periodic();
        arm.periodic();
        elevator.periodic();
        Logger.recordOutput("Arm setpoint", target.getArmAngle());
        Logger.recordOutput("wrist setpoint", target.getWristAngle());
        Logger.recordOutput("ele setpoint", target.getEleHeight());

    }

    public SuperState getState() {
        return current;
    }

    public void setTarget(SuperState goal) {
        target = goal;
    } 

    private static double applyKs(double volts, double kS, double kSDeadband) {
        if (Math.abs(volts) < kSDeadband) {
          return volts;
        }
        return volts + Math.copySign(kS, volts);
      }
    
    
}
