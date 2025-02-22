
package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;
    private final ElevatorFeedforward ff =  Constants.ElevatorConstants.SimEleFF; //Constants.ElevatorConstants.EleFF
    private final ProfiledPIDController pid = Constants.ElevatorConstants.SimElePID; //Constants.ElevatorConstants.ElePID

    public double target = 0;

    public Elevator(ElevatorIO io){
        this.io = io;
        inputs = new ElevatorIOInputsAutoLogged();
    }

    @Override
    public void periodic(){
        io.runVolts(pid.calculate(Units.metersToInches(getHeight()), Units.metersToInches(target)) + ff.calculate(20));
        io.updateInputs(inputs);
        
        Logger.processInputs("Elevator", inputs);   
    }

    public void setHeight(double target) {
        this.target = target;
    } 

    public void runVolts(double volts) {
        io.runVolts(volts);
    }

    public void setPower(double power) {
        io.setPower(power);
    }

    @AutoLogOutput(key = "Elevator/Height Meters")
    public double getHeight(){
        return io.getHeight();
    }

    //TODO: add autologoutput
    public boolean atGoal() {
        return Math.abs(getHeight() - target) < ElevatorConstants.kTolerance;
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
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.quasistatic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0),
                
            routine.dynamic(Direction.kForward)
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.dynamic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0)
        );
    }
}
