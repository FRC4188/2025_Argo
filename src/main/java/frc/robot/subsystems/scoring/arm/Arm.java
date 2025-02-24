package frc.robot.subsystems.scoring.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    public Arm(ArmIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

   public void runVolts(double volts) {
    io.runVolts(volts);
   }

    @AutoLogOutput(key = "Arm/Angle Radians")
    public double getAngle() {
        return io.getAngle();
    }
    
    @AutoLogOutput(key = "Arm/isAtSetpoint")
    public boolean atGoal(double target) {
        return Math.abs(getAngle() - target) < Constants.ArmConstants.kTolerance;
    }
    
    public Command runSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Seconds),
                Volts.of(3),
                Seconds.of(6)
            ),new SysIdRoutine.Mechanism(
                voltage -> io.runVolts(voltage.magnitude()),
                state -> SignalLogger.writeString("SysId_Arm", state.toString()),
                this));
        
        return Commands.sequence(
            routine.quasistatic(Direction.kForward)
                .until(() -> getAngle() >= SuperConstraints.ArmConstraints.HIGHEST_A),
            routine.quasistatic(Direction.kReverse)
                .until(() -> getAngle() <= SuperConstraints.ArmConstraints.LOWEST_A),
                
            routine.dynamic(Direction.kForward)
                .until(() -> getAngle() >= SuperConstraints.ArmConstraints.HIGHEST_A),
            routine.dynamic(Direction.kReverse)
                .until(() -> getAngle() <= SuperConstraints.ArmConstraints.LOWEST_A)
        );
    }
}