package frc.robot.subsystems.scoring.superstructure;
import java.time.LocalDate;

import org.littletonrobotics.junction.Logger;
import org.opencv.video.KalmanFilter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.robot;
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.superstructure.SuperToTest;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.arm.ArmIOReal;
import frc.robot.subsystems.scoring.arm.ArmIOSim;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.scoring.elevator.ElevatorIOReal;
import frc.robot.subsystems.scoring.elevator.ElevatorIOSim;
import frc.robot.subsystems.scoring.superstructure.SuperState.*;
import frc.robot.subsystems.scoring.wrist.Wrist;
import frc.robot.subsystems.scoring.wrist.WristIO;
import frc.robot.subsystems.scoring.wrist.WristIOReal;
import frc.robot.subsystems.scoring.wrist.WristIOSim;

public class Superstructure extends SubsystemBase{
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
            0.1, 0, 0,
            constraints);

    

    private ProfiledPIDController elePID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);
        
    private SuperState target;
    private SuperState current;
    
    public Superstructure(Mode mode){
        switch (mode) {
            case REAL:
                this.arm = new Arm(new ArmIOReal());
                this.wrist = new Wrist(new WristIOReal());
                this.elevator = Elevator.getInstance(new ElevatorIOReal());
                break;
            case SIM: 
                this.arm = new Arm(new ArmIOSim());
                this.wrist = new Wrist(new WristIOSim());
                this.elevator = Elevator.getInstance(new ElevatorIOSim());
                break;
            default: 
                this.arm = new Arm(new ArmIOSim());
                this.wrist = new Wrist(new WristIOSim());
                this.elevator = Elevator.getInstance(new ElevatorIOSim());
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

        elevator.runVoltsNC(
            elePID.calculate(elevator.getHeight(), target.getEleHeight())
        );

        wrist.runVolts(
            wristPID.calculate(wrist.getAngle(), target.getWristAngle())       
            
        );

        
        wrist.periodic();
        arm.periodic();
        elevator.periodic();
        Logger.recordOutput("Arm setpoint", target.getArmAngle());
        Logger.recordOutput("wrist setpoint", target.getWristAngle());
        Logger.recordOutput("ele setpoint", target.getEleHeight());

    }

    private static double applyKs(double volts, double kS, double kSDeadband) {
        if (Math.abs(volts) < kSDeadband) {
          return volts;
        }
        return volts + Math.copySign(kS, volts);
      }
    
    
}