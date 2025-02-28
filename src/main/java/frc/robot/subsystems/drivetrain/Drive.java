package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.gyro.GyroIO;
import frc.robot.subsystems.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.gyro.PhoenixOdometryThread;
import frc.robot.subsystems.vision.Limelight.VisionConsumer;
import frc.robot.util.AllianceFlip;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Drive extends SubsystemBase implements VisionConsumer {

    public PIDController translation =
     new PIDController(0, 0, 0), 
     rotation = new PIDController(0, 0, 0);

    LoggedNetworkNumber t_p = new LoggedNetworkNumber("DriveTune/tp");
    LoggedNetworkNumber t_i = new LoggedNetworkNumber("DriveTune/ti");
    LoggedNetworkNumber t_d = new LoggedNetworkNumber("DriveTune/td");

    LoggedNetworkNumber r_p = new LoggedNetworkNumber("DriveTune/rp");
    LoggedNetworkNumber r_i = new LoggedNetworkNumber("DriveTune/ri");
    LoggedNetworkNumber r_d = new LoggedNetworkNumber("DriveTune/rd");

    LoggedNetworkNumber t_target = new LoggedNetworkNumber("DriveTune/ttarget", 0);
    LoggedNetworkNumber r_target = new LoggedNetworkNumber("DriveTune/rtarget", 0);

    // TunerConstants doesn't include these constants, so they are declared locally
    public static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 54.34;
    private static final double ROBOT_MOI = 8;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    WHEEL_COF));

    public static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetOdometryCallBack;

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO, Consumer<Pose2d> resetOdometryCallBack) {
        this.gyroIO = gyroIO;
        this.resetOdometryCallBack = resetOdometryCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds), 
                        Volts.of(3), 
                        Seconds.of(5), 
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        //manual tuning

        // translation.setP(t_p.get());
        // translation.setI(t_i.get());
        // translation.setD(t_d.get());

        
        // rotation.setP(r_p.get());
        // rotation.setI(r_i.get());
        // rotation.setD(r_d.get());

        // Logger.recordOutput("Drive/r_p", rotation.getP());

        // ChassisSpeeds speeds = 
        //     ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        //        translation.calculate(Math.hypot(
        //         getChassisSpeeds().vxMetersPerSecond, 
        //         getChassisSpeeds().vyMetersPerSecond), t_target.get()),
        //       0, 
        //       rotation.calculate(getChassisSpeeds().omegaRadiansPerSecond,r_target.get())), 
        //       AllianceFlip.apply(getRotation()));
        
        // Logger.recordOutput("Drive/tvolts",  translation.calculate(Math.hypot(
        //         getChassisSpeeds().vxMetersPerSecond, 
        //         getChassisSpeeds().vyMetersPerSecond), t_target.get()));

        // Logger.recordOutput("Drive/rvolts",  
        //         rotation.calculate(getChassisSpeeds().omegaRadiansPerSecond,r_target.get()));

        // runVelocity(speeds);

        // Logger.recordOutput("Drive/Speeds", 
        //         Math.hypot(
        //         getChassisSpeeds().vxMetersPerSecond, 
        //         getChassisSpeeds().vyMetersPerSecond));

        // Logger.recordOutput("Drive/Rotations",
        //         getChassisSpeeds().omegaRadiansPerSecond);

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.robot.currMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
        
        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified by
     * {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());
        Logger.recordOutput("Drive/OdometryPose", getState().Pose);
        Logger.recordOutput("Drive/TargetStates", getState().ModuleTargets);
        Logger.recordOutput("Drive/MeasuredStates", getState().ModuleStates);
        Logger.recordOutput("Drive/MeasuredSpeeds", getState().Speeds);
        if (mapleSimSwerveDrivetrain != null)
        Logger.recordOutput("Drive/SimulationPose", mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(115),
                Inches.of(30),
                Inches.of(30),
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                1.2,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.1); // wait for simulation to update
        super.resetPose(pose);
    }
}