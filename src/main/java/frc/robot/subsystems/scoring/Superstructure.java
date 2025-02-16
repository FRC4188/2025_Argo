package frc.robot.subsystems.scoring;
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
import frc.robot.commands.autos.AutoTests;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.superstructure.SuperToTest;
import frc.robot.subsystems.scoring.SuperState.*;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.wrist.Wrist;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final SuperVisualizer sim;

    ArmFF ff;

    private final Constraints constraints = new Constraints(960.0, 720.0);

    private ProfiledPIDController armPID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);

    private PIDController wristPID = 
        new PIDController(
            0.1, 0, 0);

    private ProfiledPIDController elePID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);
        
    private SuperState target;
    private SuperState current;
    
    public Superstructure(Arm arm, Elevator elevator, Wrist wrist){
        target = SuperPreset.START.getState();
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
        sim = new SuperVisualizer("Superstructure Sim");
        ff = new ArmFF();
        this.current = new SuperState(
            wrist.getAngle(),
            arm.getAngle(),
            elevator.getHeight(), false);
    }

    public void setgoal(SuperState goal){
        target = goal;
    }

    // public void setSystem(double wristvolt, double armvolt, double elevolt){
    //     wrist.runVolts(wristvolt);
    //     arm.runVolt(armvolt);
    //     elevator.runVolts(elevolt);
    // }

    @Override
    public void periodic(){
        // var endPos = target.getEndPos();
        //var start = target;
        // current = new SuperState(
        //     wrist.getAngle(),
        //     arm.getAngle(),
        //     elevator.getHeight(), current.isCoral());
        var ffVolt = ff.calculate(
            VecBuilder.fill(target.getArmAngle(), target.getWristAngle())
        );

        arm.runVolt(
            armPID.calculate(arm.getAngle(), target.getArmAngle())
            + ffVolt.get(0, 0)
        );

        // didnt know i had to finish this class mb ig
        elevator.runVoltsNC(
            elePID.calculate(elevator.getHeight(), target.getHeight())
        );
        wrist.runVoltsNC(
            wristPID.calculate(wrist.getAngle(), target.getWristAngle())       
            + ffVolt.get(1,0)
            // Hopefully this is the right FF arguemnts for the wrist
        );

            Vector<N4> simstate = ff.simState(VecBuilder.fill(
                arm.getAngle(), wrist.getAngle(), arm.getVel(), wrist.getVel()),
            VecBuilder.fill(arm.getVolt(), wrist.getVolt()), Constants.robot.loopPeriodSecs);

            SuperState eh = new SuperState(new Translation3d( simstate.get(0,0), simstate.get(1,0), target.getHeight()), target.isCoral());

            sim.update(eh.getHeight(), eh.getArmAngle(), eh.getWristAngle());
            // sim.update(elevator.getHeight(), arm.getAngle(), wrist.getAngle());
        
        wrist.periodic();
        arm.periodic();
        elevator.periodic();
        Logger.recordOutput("Arm setpoint", target.getArmAngle());
        Logger.recordOutput("wrist setpoint", target.getWristAngle());
        Logger.recordOutput("ele setpoint", target.getHeight());

    }

    private static double applyKs(double volts, double kS, double kSDeadband) {
        if (Math.abs(volts) < kSDeadband) {
          return volts;
        }
        return volts + Math.copySign(kS, volts);
      }
    
    
}//change