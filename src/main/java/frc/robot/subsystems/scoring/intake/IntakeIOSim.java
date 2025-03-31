package frc.robot.subsystems.scoring.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private double applied_volts = 0;

    public IntakeIOSim(){
        applied_volts = 0;
        
    }

    @Override
    public void runVolts(double volts) {
        applied_volts = volts;
    }

    public boolean isIn(){
        return false; //true when laser can hit breaker aka nothing in intake
    }

    @Override
    public boolean isStalled() {
        return false;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = applied_volts;
       
    }
    
}