package frc.robot.subsystems.scoring.superstructure;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.elevator.ElevatorIO;
import frc.robot.subsystems.scoring.elevator.ElevatorIOReal;
import frc.robot.subsystems.scoring.elevator.ElevatorIOSim;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.WristConstraints;
import frc.robot.subsystems.scoring.wrist.Wrist;
import frc.robot.subsystems.scoring.wrist.WristIO;
import frc.robot.subsystems.scoring.wrist.WristIOReal;
import frc.robot.subsystems.scoring.wrist.WristIOSim;

public class Superstructure extends SubsystemBase{
    private final Elevator elevator;
    private final Wrist wrist;
    
    @AutoLogOutput (key = "Copilot/Elevator PID")
    public boolean ele_pid = true;

    @AutoLogOutput (key = "Copilot/Elevator Manual")
    public boolean ele_man = false;

    @AutoLogOutput (key = "Copilot/Wrist PID")
    public boolean wrist_pid = true;

    @AutoLogOutput (key = "Copilot/Wrist Manual")
    public boolean wrist_man = false;

    private ProfiledPIDController elePID = Constants.ElevatorConstants.ElePID; 
    private ProfiledPIDController wristPID = Constants.WristConstants.WristPID;
    private ArmFeedforward wristFF = Constants.WristConstants.WristFF; 

    private SuperState target;

    private boolean pid_tuning = false;

    LoggedNetworkNumber w_target = new LoggedNetworkNumber("WristTune/target", 0);
    LoggedNetworkNumber w_p = new LoggedNetworkNumber("WristTune/p", 0);
    LoggedNetworkNumber w_i = new LoggedNetworkNumber("WristTune/i", 0);
    LoggedNetworkNumber w_d = new LoggedNetworkNumber("WristTune/d", 0);
    LoggedNetworkNumber w_s = new LoggedNetworkNumber("WristTune/s", 0);
    LoggedNetworkNumber w_g = new LoggedNetworkNumber("WristTune/g", 0);
    LoggedNetworkNumber w_v = new LoggedNetworkNumber("WristTune/v", 0);
    LoggedNetworkNumber w_a = new LoggedNetworkNumber("WristTune/a", 0);

    LoggedNetworkNumber e_target = new LoggedNetworkNumber("EleTune/target", 0);
    LoggedNetworkNumber e_p = new LoggedNetworkNumber("EleTune/p", 0);
    LoggedNetworkNumber e_i = new LoggedNetworkNumber("EleTune/i", 0);
    LoggedNetworkNumber e_d = new LoggedNetworkNumber("EleTune/d", 0);
    LoggedNetworkNumber e_ff = new LoggedNetworkNumber("EleTune/ff", 0);

    
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

        target = new SuperState(0, 0);
        
        elePID.reset(elevator.getHeight());
        wristPID.reset(wrist.getAngle());
    }

    @Override
    public void periodic(){


        Logger.recordOutput("SuperStruct/Wrist target", target.getWristAngle());
        Logger.recordOutput("SuperStruct/Elevator target", target.getEleHeight());

        Logger.recordOutput("SuperStruct/At Target", atTarget());
        Logger.recordOutput("SuperStruct/Wrist At Target", wrist.atGoal(target.getWristAngle()));
        Logger.recordOutput("SuperStruct/Elevator At Target", elevator.atGoal(target.getEleHeight()));

        if (pid_tuning) {
            wristPID.setPID(
                w_p.get(), 
                w_i.get(), 
                w_d.get());

            wristFF = new ArmFeedforward(
                w_s.get(), 
                w_g.get(), 
                w_v.get(), 
                w_a.get());

            double wrist_volts = wristPID.calculate(wrist.getAngle(), w_target.get())
                + wristFF.calculate(wrist.getAngle() + Math.PI / 2, 0);
            
            Logger.recordOutput("WristTune/volts", wrist_volts);

            if (!wrist_pid) {
                wrist.runVolts(
                    wrist_volts
                );
            }

            elePID.setPID(
                e_p.get(), 
                e_i.get(), 
                e_d.get());

            Logger.recordOutput("EleTune/volts",elePID.calculate(elevator.getHeight(), e_target.get()));

            if (!ele_pid) {
                elevator.runVolts(
                    elePID.calculate(elevator.getHeight(), e_target.get()) 
                    + e_ff.get()
                );
            }
        } else {
            if (!wrist_man) {
                double wristvolts = 
                    wristPID.calculate(wrist.getAngle(), target.getWristAngle())
                    + wristFF.calculate(wrist.getAngle() + Math.PI / 2, 0);
                
                wrist.runVolts((wrist_pid)?wristvolts:0);
            }
                  

            if (!ele_man) {
                double elevolts = 
                    elePID.calculate(elevator.getHeight(), target.getEleHeight()) 
                    + Constants.ElevatorConstants.kFF;

                elevator.runVolts((ele_pid)?elevolts:0);
            }
        }
    }

    public void disable_manual() {
        wrist_man = false;
        ele_man = false;
    }

    public Command manual(DoubleSupplier wristinput, DoubleSupplier eleinput) {
        return Commands.run(
            ()-> {
                double wristvolts = 0;

                if (wristinput.getAsDouble() == 0) {
                    wrist_man = false;
                } else {
                    wrist_man = true;

                    if (wrist_pid) {
                        wristvolts = MathUtil.clamp(5 * wristinput.getAsDouble() + wristFF.calculate(wrist.getAngle() + Math.PI / 2, 0), 
                        (wrist.getAngle() < WristConstraints.LOWEST_A)?0:-7, 
                        (wrist.getAngle() > WristConstraints.HIGHEST_A)?0:7);
                    } else {
                        wristvolts = MathUtil.clamp(5 * wristinput.getAsDouble(), 
                        -5, 5);
                    }
                    
                    target.setWristAngle(
                        MathUtil.clamp(
                            wrist.getAngle(), 
                            SuperConstraints.WristConstraints.LOWEST_A, 
                            SuperConstraints.WristConstraints.HIGHEST_A));
        
                    wristPID.reset(wrist.getAngle());
                    wrist.runVolts(wristvolts);
                }
        
                double elevolts = 0;
        
                if (eleinput.getAsDouble() == 0) {
                    ele_man = false;
                } else {    
                    ele_man = true;

                    elevolts = MathUtil.clamp(
                        Constants.ElevatorConstants.kFF - 7 * eleinput.getAsDouble(), 
                        0, 
                        (elevator.getHeight() >= SuperConstraints.ElevatorConstraints.RANGE)?
                            Constants.ElevatorConstants.kFF:12);
        
                    target.setEleHeight(
                        MathUtil.clamp(
                            elevator.getHeight(), 
                            0, 
                            SuperConstraints.ElevatorConstraints.RANGE));
                    
                    elevator.runVolts(elevolts);
                    elePID.reset(elevator.getHeight());
                }
            }, 
        this
        );
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

    public double getWristAngle() {
        return wrist.getAngle();
    }  

    public double getEleHeight() {
        return elevator.getHeight();
    }

    public void setTarget(SuperState goal) {
        target = goal;
    }  

    public void setWrist(double angle) {
        target.setWristAngle(angle);
    }

    public void setEle(double height) {
        target.setEleHeight(height);
    }
}
