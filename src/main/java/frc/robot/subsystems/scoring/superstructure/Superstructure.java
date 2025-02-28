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
    
    @AutoLogOutput (key = "Elevator Manual Override")
    public boolean eleOverride = false;

    @AutoLogOutput (key = "Arm Manual Override")
    public boolean armOverride = false;

    @AutoLogOutput (key = "Wrist Manual Override")
    public boolean wristOverride = false;

    private ProfiledPIDController elePID = Constants.ElevatorConstants.ElePID; //Constants.ElevatorConstants.SimElePID
    private ProfiledPIDController wristPID = Constants.WristConstants.WristPID; //Constants.WristConstants.SimWristPID
    private ProfiledPIDController armPID = Constants.ArmConstants.ArmPID; //Constants.ArmConstants.SimArmPID

    private ArmFeedforward armFF = Constants.ArmConstants.ArmFF; //Constants.ArmConstants.SimArmFF;
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

    // LoggedNetworkNumber a_target = new LoggedNetworkNumber("ArmTune/target", 0);
    // LoggedNetworkNumber a_p = new LoggedNetworkNumber("ArmTune/p", 0);
    // LoggedNetworkNumber a_i = new LoggedNetworkNumber("ArmTune/i", 0);
    // LoggedNetworkNumber a_d = new LoggedNetworkNumber("ArmTune/d", 0);
    // LoggedNetworkNumber a_s = new LoggedNetworkNumber("ArmTune/s", 0);
    // LoggedNetworkNumber a_g = new LoggedNetworkNumber("ArmTune/g", 0);
    // LoggedNetworkNumber a_v = new LoggedNetworkNumber("ArmTune/v", 0);
    // LoggedNetworkNumber a_a = new LoggedNetworkNumber("ArmTune/a", 0);

    // LoggedNetworkNumber e_target = new LoggedNetworkNumber("EleTune/target", 0);
    // LoggedNetworkNumber e_p = new LoggedNetworkNumber("EleTune/p", 0);
    // LoggedNetworkNumber e_i = new LoggedNetworkNumber("EleTune/i", 0);
    // LoggedNetworkNumber e_d = new LoggedNetworkNumber("EleTune/d", 0);
    // LoggedNetworkNumber e_ff = new LoggedNetworkNumber("EleTune/ff", 0);

    
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
        //sim stuff
        // SuperState current = new SuperState(
        //     wrist.getAngle(),
        //     arm.getAngle(),
        //     elevator.getHeight());

        // sim.update(current);

        Logger.recordOutput("SuperStruct/Arm target", target.getArmAngle());
        Logger.recordOutput("SuperStruct/Wrist target",target.getWristAngle());
        Logger.recordOutput("SuperStruct/Elevator target", target.getEleHeight());

        Logger.recordOutput("SuperStruct/At Target", atTarget());
        Logger.recordOutput("SuperStruct/Wrist At Target", wrist.atGoal(target.getWristAngle()));
        Logger.recordOutput("SuperStruct/Arm At Target", arm.atGoal(target.getArmAngle()));
        Logger.recordOutput("SuperStruct/Elevator At Target", elevator.atGoal(target.getEleHeight()));

        //PID tuning so ill kill myself later

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
        // + wristFF.calculate(wrist.getAngle() + arm.getAngle() + Math.PI / 2, 0);
        // Logger.recordOutput("WristTune/volts", wrist_volts);

        // if (!wristOverride) {
        //     wrist.runVolts(
        //         wristPID.calculate(wrist.getAngle(), w_target.get())
        //         + wristFF.calculate(wrist.getAngle() + arm.getAngle() + Math.PI / 2, 0)
        //         );
        // }

        // armPID.setPID(
        //     a_p.get(), 
        //     a_i.get(), 
        //     a_d.get());

        // armFF = new ArmFeedforward(
        //     a_s.get(), 
        //     a_g.get(), 
        //     a_v.get(), 
        //     a_a.get());

        // Logger.recordOutput("pid volts", armPID.calculate(arm.getAngle(), a_target.get()));
        // Logger.recordOutput("ff volts", armFF.calculate(arm.getAngle() + Math.PI / 2, 0));

        // if (!armOverride) {
        //     arm.runVolts(
        //         armPID.calculate(arm.getAngle(), a_target.get())
        //         + armFF.calculate(arm.getAngle() + Math.PI / 2, 0)
        //     );
        // }

        // elePID.setPID(
        //     e_p.get(), 
        //     e_i.get(), 
        //     e_d.get());

        // //Logger.recordOutput("pid volts",elePID.calculate(elevator.getHeight(), e_target.get()));

        // if (!eleOverride) {
        //     elevator.runVolts(
        //         elePID.calculate(elevator.getHeight(), e_target.get()) 
        //         + e_ff.get()
        //     );
        // }


    }

    public void manualOverride(DoubleSupplier wristinput, DoubleSupplier arminput, DoubleSupplier eleinput) {
        if (pidOverride) return;

        double wristvolts = 0;
        Logger.recordOutput("Wrist/pid?", Math.abs(wristinput.getAsDouble()) < 0.1 && !wristOverride);

        if (Math.abs(wristinput.getAsDouble()) < 0.1 && !wristOverride) {
            wristvolts = 
                wristPID.calculate(wrist.getAngle(), target.getWristAngle())
                + wristFF.calculate(wrist.getAngle() + arm.getAngle() + Math.PI / 2, 0);
        } else {
            wristvolts = MathUtil.clamp(wristinput.getAsDouble(), -12, 12);
            target.setWristAngle(
                MathUtil.clamp(
                    wrist.getAngle(), 
                    SuperConstraints.WristConstraints.LOWEST_A, 
                    SuperConstraints.WristConstraints.HIGHEST_A));
        }

        double armvolts = 0;

        Logger.recordOutput("Arm/pid?", Math.abs(arminput.getAsDouble()) < 0.1 && !armOverride);
        if (Math.abs(arminput.getAsDouble()) < 0.1 && !armOverride) {
            armvolts = armPID.calculate(arm.getAngle(), target.getArmAngle())
                + armFF.calculate(arm.getAngle() + Math.PI / 2, 0);
        } else {
            armvolts = MathUtil.clamp(arminput.getAsDouble(), -12, 12);
            target.setArmAngle(
                MathUtil.clamp(
                    arm.getAngle(), 
                    SuperConstraints.ArmConstraints.LOWEST_A, 
                    SuperConstraints.ArmConstraints.HIGHEST_A));
        }

        double elevolts = 0;


        Logger.recordOutput("Elevator/pid?", Math.abs(eleinput.getAsDouble()) < 0.1 && !eleOverride);
        if (Math.abs(eleinput.getAsDouble()) < 0.1  && !eleOverride) {
            elevolts = elePID.calculate(elevator.getHeight(), target.getEleHeight()) 
            + Constants.ElevatorConstants.kFF;
        } else {
            elevolts = MathUtil.clamp(Constants.ElevatorConstants.kFF - eleinput.getAsDouble(), 0, 12);
            target.setEleHeight(elevator.getHeight());
        }

        arm.runVolts(armvolts);
        wrist.runVolts(wristvolts);
        elevator.runVolts(elevolts);
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
 
    public Command resetEle() {
        return Commands.runOnce(() -> elevator.setZero());
    }

    public Command resetWrist() {
        return Commands.runOnce(() -> wrist.setZero());
    }

    public SuperState getState() {
        return new SuperState(wrist.getAngle(), arm.getAngle(), elevator.getHeight());
    }


    public boolean atTarget() {
        return wrist.atGoal(target.getWristAngle()) && arm.atGoal(target.getArmAngle()) && elevator.atGoal(target.getEleHeight());
    }

    public boolean wristAtTarget() {
        return  wrist.atGoal(target.getWristAngle());
    }

    public boolean armAtTarget() {
        return arm.atGoal(target.getArmAngle());
    }

    public boolean eleAtTarget() {
        return elevator.atGoal(target.getEleHeight());
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

    public void setArm(double angle) {
        target.setArmAngle(angle);
    }

    public void setWrist(double angle) {
        target.setWristAngle(angle);
    }

    public void setEle(double height) {
        target.setEleHeight(height);
    }
}