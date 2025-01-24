package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOReal {

    TalonFX RightMotor;
    TalonFX LeftMotor;

    CANcoder RightEncoder;
    CANcoder LeftEncoder;

    double LeftZero = 0;
    double RightZero = 0;

    PIDController PID;
    double speedAfterPID;

    public ElevatorIOReal() {

        //TODO: Set all the device ids, 0 for now cause idk robot isnt built???
        LeftMotor = new TalonFX(0);
        LeftEncoder = new CANcoder(0);
        RightMotor = new TalonFX(0);
        RightEncoder = new CANcoder(0);

        //TODO: Tune this   
        PID = new PIDController(0.0, 0.0, 0.0);
        PID.enableContinuousInput(-180.0, 180.0);
        PID.setTolerance(0, 0);


        LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.5;
        LeftMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;
        LeftMotor.getConfigurator().apply(closedLoopRampsConfigs);
        RightMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigsAngle = new OpenLoopRampsConfigs();
        openLoopRampsConfigsAngle.VoltageOpenLoopRampPeriod = 0.5;
        RightMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigsAngle = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigsAngle.VoltageClosedLoopRampPeriod = 0.5;
        RightMotor.getConfigurator().apply(closedLoopRampsConfigs);
        RightMotor.setInverted(true);

        LeftMotor.clearStickyFaults();
        RightMotor.clearStickyFaults();
        LeftEncoder.clearStickyFaults();
        RightEncoder.clearStickyFaults();

        MagnetSensorConfigs leftSensorConfigs = new MagnetSensorConfigs();
        leftSensorConfigs.MagnetOffset = -(LeftZero / 360.0);
        leftSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        MagnetSensorConfigs rightSensorConfigs = new MagnetSensorConfigs();
        rightSensorConfigs.MagnetOffset = -(LeftZero / 360.0);
        rightSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        LeftEncoder.getConfigurator().apply(leftSensorConfigs);
        LeftMotor.getConfigurator().setPosition(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        RightEncoder.getConfigurator().apply(rightSensorConfigs);
        RightMotor.getConfigurator().setPosition(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        LeftMotor.optimizeBusUtilization();
        RightMotor.optimizeBusUtilization();    



        
    }

    public void manualPower(double speed) {
        LeftMotor.set(speed);
        RightMotor.set(speed);
    }

    public void stopElevator() {
        // Stop the elevator
        LeftMotor.set(0.0);
        RightMotor.set(0.0);
        
        double leftPosition = LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        double rightPosition = RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;



        LeftMotor.set(PID.calculate(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,leftPosition));
        RightMotor.set(PID.calculate(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,rightPosition));

    }

    public void setElevatorPosition(double position) {
        LeftMotor.set(PID.calculate(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,position));
        RightMotor.set(PID.calculate(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,position));
    }

    public double getElevatorPosition() {
        
        return (LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)+(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)/2.0;
    }

    public void zeroElevator() {
        
    }

    public void periodic() {
        
    }
    
}
