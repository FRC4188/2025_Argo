package frc.robot.subsystems.scoring.superstructure;


import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.arm.ArmIO;
import frc.robot.subsystems.scoring.arm.ArmIOReal;
import frc.robot.subsystems.scoring.arm.ArmIOSim;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.elevator.ElevatorIO;
import frc.robot.subsystems.scoring.elevator.ElevatorIOReal;
import frc.robot.subsystems.scoring.elevator.ElevatorIOSim;
import frc.robot.subsystems.scoring.intake.Intake;
import frc.robot.subsystems.scoring.wrist.Wrist;
import frc.robot.subsystems.scoring.wrist.WristIO;
import frc.robot.subsystems.scoring.wrist.WristIOReal;
import frc.robot.subsystems.scoring.wrist.WristIOSim;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;

    private SuperVisualizer sim;
    
    public boolean eleOverride = true;
    public boolean armOverride = true;
    public boolean wristOverride = true;

    private ProfiledPIDController elePID = Constants.ElevatorConstants.ElePID; //Constants.ElevatorConstants.SimElePID
    private ProfiledPIDController wristPID = Constants.WristConstants.WristPID; //Constants.WristConstants.SimWristPID
    private ProfiledPIDController armPID = Constants.ArmConstants.ArmPID; //Constants.ArmConstants.SimArmPID

    private ElevatorFeedforward eleFF = Constants.ElevatorConstants.EleFF; //Constants.ElevatorConstants.SimEleFF
    private ArmFeedforward armFF = Constants.ArmConstants.ArmFF; //Constants.ArmConstants.SimArmFF;
    private ArmFeedforward wristFF = Constants.WristConstants.WristFF; //Constants.WristConstants.SimWristFF

    private SuperState target;
    
    public Superstructure(Mode mode){
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
                this.arm = new Arm(new ArmIO() {});
                this.wrist = new Wrist(new WristIO() {});
                this.elevator = new Elevator(new ElevatorIO() {});
        }
        sim = new SuperVisualizer("Superstructure");

        target = new SuperState(
            wrist.getAngle(),
            arm.getAngle(),
            elevator.getHeight());
    }

    @Override
    public void periodic(){
        SuperState current = new SuperState(
            wrist.getAngle(),
            arm.getAngle(),
            elevator.getHeight());

        sim.update(current);

        if (!wristOverride) {
            wrist.runVolts(
                wristPID.calculate(wrist.getAngle(), target.getWristAngle())
                + wristFF.calculate(target.getGlobalAngle() + Math.PI / 2, 0)
                );
        }

        if (!armOverride) {
            arm.runVolts(
                armPID.calculate(arm.getAngle(), target.getArmAngle())
                + armFF.calculate(target.getArmAngle() + Math.PI / 2, 0)
            );
        }

        if (!eleOverride) {
            elevator.runVolts(
                elePID.calculate(Units.metersToInches(elevator.getHeight()), Units.metersToInches(target.getEleHeight())) 
                + eleFF.calculate(target.getEleHeight(), 0)
            );
        }
        Logger.recordOutput("Arm manual", armOverride);
        Logger.recordOutput("wrist manual",wristOverride);
        Logger.recordOutput("ele manual", eleOverride);

        Logger.recordOutput("At Target", atTarget());
    }

    //sysid
    public Command armSysId() {
        return arm.runSysId();
    }

    public Command wristSysId() {
        return wrist.runSysId();
    }

    public Command elevatorSysId() {
        return elevator.runSysId();
    }
 

    public Command runArm(Supplier<Double> volts) {
        return Commands.run(() -> arm.runVolts(volts.get())).onlyIf(() -> armOverride);
    }

    public Command runWrist(Supplier<Double> volts) {
        return Commands.run(() -> wrist.runVolts(volts.get())).onlyIf( () -> wristOverride);
    }

    public Command runEle(Supplier<Double> volts) {
        return Commands.run(() -> elevator.runVolts(volts.get())).onlyIf( () -> eleOverride);
    }

    public SuperState getState() {
        return new SuperState(wrist.getAngle(), arm.getAngle(), elevator.getHeight());
    }


    public boolean atTarget() {
        return wrist.atGoal(target.getWristAngle()) && arm.atGoal(target.getArmAngle()) && elevator.atGoal(target.getEleHeight());
    }

    public boolean setTarget(SuperState goal) {
        if (Intake.intakeState == Intake.Mode.ALGAE) {
            Translation2d g_cartesian = goal.getCartesian(false);
            Translation2d c_cartesian = goal.getCartesian(false);

            if (c_cartesian.getX() > 0 && g_cartesian.getX() <= Units.inchesToMeters(12) || 
                c_cartesian.getX() < 0 && g_cartesian.getX() >= Units.inchesToMeters(-12)) {
                    return false;
                }
        }

        target = goal;

        return true;
    }  
}