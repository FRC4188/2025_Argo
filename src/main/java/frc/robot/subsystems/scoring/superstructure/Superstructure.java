package frc.robot.subsystems.scoring.superstructure;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
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
    private final Elevator elevator;
    private final Wrist wrist;

    // private SuperVisualizer sim;
    
    @AutoLogOutput (key = "Copilot/Elevator Manual Override")
    public boolean eleOverride = false;

    @AutoLogOutput (key = "Copilot/Wrist Manual Override")
    public boolean wristOverride = false;

    private ProfiledPIDController elePID = Constants.ElevatorConstants.ElePID; //Constants.ElevatorConstants.SimElePID
    private ProfiledPIDController wristPID = Constants.WristConstants.WristPID; //Constants.WristConstants.SimWristPID
    private ArmFeedforward wristFF = Constants.WristConstants.WristFF; //Constants.WristConstants.SimWristFF

    private SuperState target;

    private boolean pidOverride = false;

    // LoggedNetworkNumber w_target = new LoggedNetworkNumber("WristTune/target", 0);
    // LoggedNetworkNumber w_p = new LoggedNetworkNumber("WristTune/p", 0);
    // LoggedNetworkNumber w_i = new LoggedNetworkNumber("WristTune/i", 0);
    // LoggedNetworkNumber w_d = new LoggedNetworkNumber("WristTune/d", 0);
    // LoggedNetworkNumber w_s = new LoggedNetworkNumber("WristTune/s", 0);
    // LoggedNetworkNumber w_g = new LoggedNetworkNumber("WristTune/g", 0);
    // LoggedNetworkNumber w_v = new LoggedNetworkNumber("WristTune/v", 0);
    // LoggedNetworkNumber w_a = new LoggedNetworkNumber("WristTune/a", 0);

    // LoggedNetworkNumber e_target = new LoggedNetworkNumber("EleTune/target", 0);
    // LoggedNetworkNumber e_p = new LoggedNetworkNumber("EleTune/p", 0);
    // LoggedNetworkNumber e_i = new LoggedNetworkNumber("EleTune/i", 0);
    // LoggedNetworkNumber e_d = new LoggedNetworkNumber("EleTune/d", 0);
    // LoggedNetworkNumber e_ff = new LoggedNetworkNumber("EleTune/ff", 0);

    
    public Superstructure(Mode mode){
        switch (mode) {
            case REAL:
                this.wrist = new Wrist(new WristIOReal());
                this.elevator = new Elevator(new ElevatorIOReal());
                break;
            case SIM: 
                this.wrist = new Wrist(new WristIOSim());
                this.elevator = new Elevator(new ElevatorIOSim());
                break;
            default: 
                this.wrist = new Wrist(new WristIO() {});
                this.elevator = new Elevator(new ElevatorIO() {});
        }
        //sim = new SuperVisualizer("Superstructure");

        target = new SuperState(
            wrist.getAngle(),
            elevator.getHeight());

    }

    @Override
    public void periodic(){
        //sim stuff
        // SuperState current = new SuperState(
        //     wrist.getAngle(),
        //     arm.getAngle(),
        //     elevator.getHeight());

        // sim.update(current);

        Logger.recordOutput("SuperStruct/Wrist target",target.getWristAngle());
        Logger.recordOutput("SuperStruct/Elevator target", target.getEleHeight());

        Logger.recordOutput("SuperStruct/At Target", atTarget());
        Logger.recordOutput("SuperStruct/Wrist At Target", wrist.atGoal(target.getWristAngle()));
        Logger.recordOutput("SuperStruct/Elevator At Target", elevator.atGoal(target.getEleHeight()));

        // //PID tuning so ill kill myself later

        // wristPID.setPID(
        //     w_p.get(), 
        //     w_i.get(), 
        //     w_d.get());

        // wristFF = new ArmFeedforward(
        //     w_s.get(), 
        //     w_g.get(), 
        //     w_v.get(), 
        //     w_a.get());

        // double wrist_volts = wristPID.calculate(wrist.getAngle(), w_target.get())
        // + wristFF.calculate(wrist.getAngle() + Math.PI / 2, 0);
        // Logger.recordOutput("WristTune/volts", wrist_volts);

        // if (!wristOverride) {
        //     wrist.runVolts(
        //         wrist_volts
        //         );
        // }

        // elePID.setPID(
        //     e_p.get(), 
        //     e_i.get(), 
        //     e_d.get());

        // Logger.recordOutput("pid volts",elePID.calculate(elevator.getHeight(), e_target.get()));

        // if (!eleOverride) {
        //     elevator.runVolts(
        //         elePID.calculate(elevator.getHeight(), e_target.get()) 
        //         + e_ff.get()
        //     );
        // }

        // sim.update(new SuperState(wrist.getAngle(), arm.getAngle(), elevator.getHeight()));
    }

    public void manualOverride(DoubleSupplier wristinput, DoubleSupplier eleinput) {
        if (pidOverride) return;

        double wristvolts = 0;

        if (Math.abs(wristinput.getAsDouble()) < 0.1 && !wristOverride) {
            wristvolts = 
                wristPID.calculate(wrist.getAngle(), target.getWristAngle())
                + wristFF.calculate(wrist.getAngle() + Math.PI / 2, 0);
        } else {
            wristvolts = MathUtil.clamp(wristinput.getAsDouble(), -12, 12);
            
            target.setWristAngle(
                MathUtil.clamp(
                    wrist.getAngle(), 
                    SuperConstraints.WristConstraints.LOWEST_A, 
                    SuperConstraints.WristConstraints.HIGHEST_A));
        }


        double elevolts = 0;


        if (Math.abs(eleinput.getAsDouble()) < 0.1  && !eleOverride) {
            elevolts = elePID.calculate(elevator.getHeight(), target.getEleHeight()) 
                + Constants.ElevatorConstants.kFF;
        } else {
            // if needed, replace the 0 in the clamp of the next commented out line below this one to -12
    
            elevolts = MathUtil.clamp(
                Constants.ElevatorConstants.kFF - eleinput.getAsDouble(), 
                0, 
                (elevator.getHeight() >= SuperConstraints.ElevatorConstraints.RANGE)?
                    Constants.ElevatorConstants.kFF:12);

            target.setEleHeight(
                MathUtil.clamp(
                    elevator.getHeight(), 
                    0, 
                    SuperConstraints.ElevatorConstraints.RANGE));
        }

        wrist.runVolts(wristvolts);
        elevator.runVolts(elevolts);
    }

    public Command wristSysId() {
        return wrist.runSysId();
    }

    public Command elevatorSysId() {
        return elevator.runSysId();
    }
 
    public Command resetEle() {
        return Commands.runOnce(() -> elevator.setZero());
    }

    public Command resetWrist() {
        return Commands.runOnce(() -> wrist.setZero());
    }

    public SuperState getState() {
        return new SuperState(wrist.getAngle(), elevator.getHeight());
    }


    public boolean atTarget() {
        return wrist.atGoal(target.getWristAngle()) && elevator.atGoal(target.getEleHeight());
    }

    public boolean wristAtTarget() {
        return  wrist.atGoal(target.getWristAngle());
    }

    public boolean eleAtTarget() {
        return elevator.atGoal(target.getEleHeight());
    }

    public void setTarget(SuperState goal) {
        target = SuperConstraints.clamp(goal);
    }  

    public void setWrist(double angle) {
        target.setWristAngle(angle);
    }

    public void setEle(double height) {
        target.setEleHeight(height);
    }
}
