package frc.robot.subsystems.scoring.arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final ArmFeedforward ff =  Constants.ArmConstants.SimArmFF; //Constants.ArmConstants.ArmFF;
    private final ProfiledPIDController pid = Constants.ArmConstants.SimArmPID; //Constants.ArmConstants.ArmPID;

    public double target = 0;
    
    public Arm(ArmIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.runVolts(
            pid.calculate(getAngle(), target)
            + ff.calculate(getAngle() + Math.PI/2, 0));

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

   public void setTarget(double target) {
        this.target = target;
   }

    @AutoLogOutput(key = "Arm/Angle Radians")
    public double getAngle() {
        return io.getAngle();
    }
    
    @AutoLogOutput(key = "Arm/isAtSetpoint")
    public boolean atGoal() {
        return Math.abs(getAngle() - target) < Constants.ArmConstants.kTolerance;
    }
    public Command runSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(4).per(Seconds),
                Volts.of(8),
                Seconds.of(6)
            ),new SysIdRoutine.Mechanism(
                voltage -> io.runVolts(voltage.magnitude()),
                null,
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