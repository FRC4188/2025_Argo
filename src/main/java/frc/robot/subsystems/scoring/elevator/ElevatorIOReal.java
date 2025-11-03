package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.ElevatorConstants.kMotorConfig;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.Id;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.units.measure.Current;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX leader;
    private final TalonFX follower;
    
    private final StatusSignal<Angle> positionRots;
    private final StatusSignal<AngularVelocity> velocityRots;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;

    protected final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withEnableFOC(true);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0).withEnableFOC(true);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
        new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
        new VelocityTorqueCurrentFOC(0.0);

    private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOReal() {
        leader = new TalonFX(Id.kElevatorLead, Constants.robot.rio);
        follower = new TalonFX(Id.kElevatorFollow, Constants.robot.rio);

        follower.setControl(new Follower(Id.kElevatorLead, false));

        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);

        leader.clearStickyFaults();
        follower.clearStickyFaults();

        leader.getConfigurator().apply(kMotorConfig);
        follower.getConfigurator().apply(kMotorConfig);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();   
        
        positionRots = leader.getPosition();
        velocityRots = leader.getVelocity();
        appliedVolts = leader.getMotorVoltage();
        currentAmps = leader.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, positionRots, velocityRots, appliedVolts, currentAmps);
        ParentDevice.optimizeBusUtilizationForAll(leader, follower);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderConnected =
        BaseStatusSignal.refreshAll(
                positionRots, appliedVolts, velocityRots, currentAmps)
            .isOK();

        inputs.followerConnected =
        BaseStatusSignal.refreshAll(
                follower.getMotorVoltage(), follower.getStatorCurrent())
            .isOK();
    
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.positionMeters = positionRots.getValueAsDouble() / (3 * Constants.ElevatorConstants.kPitchRadius);
        inputs.velocityMeters = velocityRots.getValueAsDouble() / (3 * Constants.ElevatorConstants.kPitchRadius);
    }

    

    @Override
    public void setOpenLoop(double volts){
        volts = MathUtil.clamp(volts, -12, 12);
        leader.setControl(new VoltageOut(volts)); 
    }

    // yanshu fix

    @Override 
    public void setElevatorHeight(DoubleSupplier target) {
        leader.setControl(
            positionVoltageRequest.withPosition(target.getAsDouble())
        );
    }


    

    @Override
    public boolean isStalled() {
        return Math.abs(leader.getStatorCurrent().getValueAsDouble()) > 35;
    }

    @Override
    public double getHeight(){
        return Units.rotationsToRadians(posRots.getValueAsDouble());
    }
}