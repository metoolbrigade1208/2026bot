package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import frc.robot.subsystems.Constants;

public class Turret extends SubsystemBase {

    static public AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold
    static public Angle toleranceAngle = Degrees.of(1); // Set a threshold

    // Motor control
    SparkMax turretMotor = new SparkMax(Constants.Turret.motorID, MotorType.kBrushless);
    RelativeEncoder turretEncoder = turretMotor.getEncoder();
    SparkMaxConfig turretConfig = new SparkMaxConfig();

    // Easy CRT
    DutyCycleEncoder enc1Encoder = new DutyCycleEncoder(Constants.Turret.enc1Id);
    DutyCycleEncoder enc2Encoder = new DutyCycleEncoder(Constants.Turret.enc2Id);
    Supplier<Angle> enc1Supplier = () -> Constants.Turret.turretAngleUnit.of(enc1Encoder.get());
    Supplier<Angle> enc2Supplier = () -> Constants.Turret.turretAngleUnit.of(enc2Encoder.get());

    Angle turretCRTAngle;

    /**
     * telemetry table.
     */
    private NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry\\Turret");
    private DoubleTopic crtAngleTopic = telemetryTable.getDoubleTopic("CRTAngle");
    private DoubleTopic enc1AngleTopic = telemetryTable.getDoubleTopic("Enc1Angle");
    private DoubleTopic enc2AngleTopic = telemetryTable.getDoubleTopic("Enc2Angle");
    private DoublePublisher crtAnglePublisher = crtAngleTopic.publish();
    private DoublePublisher enc1AnglePublisher = enc1AngleTopic.publish();
    private DoublePublisher enc2AnglePublisher = enc2AngleTopic.publish();
    private DoubleTopic setPointTopic = telemetryTable.getDoubleTopic("setPoint");
    private DoublePublisher setPointPublisher = setPointTopic.publish();

    // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T
    // and 23T pinions.
    EasyCRTConfig easyCrt = new EasyCRTConfig(enc1Supplier, enc2Supplier)
            .withCommonDriveGear(
                    /* commonRatio (mech:drive) */ 1,
                    /* driveGearTeeth */ 200,
                    /* encoder1Pinion */ 19,
                    /* encoder2Pinion */ 21)
            .withAbsoluteEncoderOffsets(Constants.Turret.enc1Zero, Constants.Turret.enc2Zero) // set after mechanical
                                                                                              // zero
            .withMechanismRange(Rotations.of(-0.5), Rotations.of(0.5)) // -180 deg to +180 deg
            .withMatchTolerance(Rotations.of(0.0265)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(true, true);

    // you can inspect:
    // easyCrt.getUniqueCoverage(); // Optional<Angle> coverage from prime counts
    // and common scale
    // easyCrt.coverageSatisfiesRange(); // Does coverage exceed maxMechanismAngle?
    // easyCrt.getRecommendedCrtGearPair(); // Suggested pair within constraints

    // Create the solver (initialized in constructor so we can swap to a sim-only
    // supplier there)
    EasyCRT easyCrtSolver;

    // Simulation objects
    private final SingleJointedArmSim m_turretSim;
    private final SparkMaxSim m_turretMotorSim;
    // Visualization
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_shooterLigament;

    // Pose publishing (publish as a typed Pose3d struct so visualizers can attach
    // in 3D)
    private final NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("Pose");
    private final StructPublisher<Pose3d> turretPosePublisher = poseTable.getStructTopic("turretPose", Pose3d.struct)
            .publish();

    // Shooter velocity supplier (in angular velocity units)
    private Supplier<AngularVelocity> shooterVelocitySupplier = () -> RobotContainer.shooter.getVelocity();
    // Turret mounting Pose3d (relative to robot origin). Default comes from
    // Constants.kTurretOffset.
    private Pose3d turretMountPose = Constants.Turret.kTurretOffset;
    private Transform2d offsetTransform = new Transform2d(turretMountPose.getTranslation().toTranslation2d(),
            Rotation2d.kZero);
    private Rotation2d mountYaw = turretMountPose.getRotation().toRotation2d();

    public Turret() {

        // Setup visualization
        m_mech2d = new Mechanism2d(60, 60);
        m_root = m_mech2d.getRoot("TurretRoot", 30, 30);
        // Shooter visualization ligament (length will be updated in periodic)
        m_shooterLigament = m_root.append(new MechanismLigament2d("ShooterSpeed", 10, 0, 0,
                new Color8Bit(Color.kBlue)));
        SmartDashboard.putData("Turret Sim", m_mech2d);

        // Setup simulation objects
        // Use a single NEO motor model and a 30:1 gearing (motor:mech)
        final DCMotor turretGearbox = DCMotor.getNEO(1);
        final double gearingRatio = 30.0;
        // Estimate a small moment of inertia: radius 0.2 m, mass 5 kg (conservative)
        final double mechRadius = 0.2;
        final double mechMass = 5.0;
        final double mechMOI = SingleJointedArmSim.estimateMOI(mechRadius, mechMass);

        m_turretSim = new SingleJointedArmSim(turretGearbox, gearingRatio, mechMOI, mechRadius,
                Constants.Turret.revLimit.in(Radian), Constants.Turret.fwdLimit.in(Radian), false,
                Units.degreesToRadians(0));

        m_turretMotorSim = new SparkMaxSim(turretMotor, turretGearbox);

        // Create the CRT solver. In simulation we will provide simulated
        // absolute-encoder suppliers which are derived from the turret simulation state
        // so the CRT solver sees physically-correct encoder readings that reflect
        // gearing differences.
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            Supplier<Angle> simEnc1Supplier = () -> {
                double mechRot = Units.radiansToRotations(m_turretSim.getAngleRads());
                double encRot = Constants.Turret.enc1Zero.in(Rotations) + mechRot * (200.0 / 19.0);
                return Rotations.of(encRot);
            };
            Supplier<Angle> simEnc2Supplier = () -> {
                double mechRot = Units.radiansToRotations(m_turretSim.getAngleRads());
                double encRot = Constants.Turret.enc2Zero.in(Rotations) + mechRot * (200.0 / 23.0);
                return Rotations.of(encRot);
            };

            EasyCRTConfig simCfg = new EasyCRTConfig(simEnc1Supplier, simEnc2Supplier)
                    .withCommonDriveGear(
                            /* commonRatio (mech:drive) */ 1,
                            /* driveGearTeeth */ 200,
                            /* encoder1Pinion */ 19,
                            /* encoder2Pinion */ 23)
                    .withAbsoluteEncoderOffsets(Constants.Turret.enc1Zero, Constants.Turret.enc2Zero)
                    .withMechanismRange(Rotations.of(-0.5), Rotations.of(0.5))
                    .withMatchTolerance(Rotations.of(0.0265))
                    .withAbsoluteEncoderInversions(false, false);

            easyCrtSolver = new EasyCRT(simCfg);
        } else {
            easyCrtSolver = new EasyCRT(easyCrt);
        }

        turretConfig
                .closedLoopRampRate(.25)
                .openLoopRampRate(.25)
                .smartCurrentLimit(20) // Neo550
                .idleMode(IdleMode.kBrake)
                .encoder
                .positionConversionFactor(Constants.Turret.gearing.in(Constants.Turret.turretAngleUnit))
                .velocityConversionFactor(Constants.Turret.gearSpeed.in(Constants.Turret.turretAngleUnit.per(Minute))); 
                // default is RPM, not RPS
        turretConfig.softLimit
                .forwardSoftLimit(Constants.Turret.fwdLimit.in(Constants.Turret.turretAngleUnit))
                .reverseSoftLimit(Constants.Turret.revLimit.in(Constants.Turret.turretAngleUnit))
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        turretConfig.closedLoop
                .pid(Constants.Turret.kP, 0, Constants.Turret.kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .allowedClosedLoopError(toleranceAngle.in(Rotations), ClosedLoopSlot.kSlot0).feedForward
                .sv(Constants.Turret.kS.baseUnitMagnitude(), Constants.Turret.kV.magnitude());
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        easyCrtSolver.getAngleOptional().ifPresent((angle) -> {
            turretMotor.getEncoder().setPosition(angle.in(Rotations));
        });
    }

    public Angle getAngle() {
        return Constants.Turret.turretAngleUnit.of(turretMotor.getEncoder().getPosition()); // Get the current angle of
                                                                                            // the turret
    }

    public void simulationPeriodic() {
        // Update the simulated turret model. Use the motor applied output ([-1,1])
        // times the current battery voltage as the input.
        m_turretSim.setInput(turretMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Update the SparkMax simulation (provides simulated internal encoder values)
        m_turretMotorSim.iterate(m_turretSim.getVelocityRadPerSec(),
                RoboRioSim.getVInVoltage(), 0.020);

        // Step the mechanism simulation forward one robot loop (20ms)
        m_turretSim.update(0.020);

        //turretEncoder.setPosition(new Rotation2d(m_turretSim.getAngleRads()).getRadians());

        // Update simulated battery voltage based on current draw
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretSim.getCurrentDrawAmps()));
                
                
    }

    public void periodic() {
        // set the current CRT angle and publish it
        // TODO: use it to set the current turret readout angle
        easyCrtSolver.getAngleOptional().ifPresent((crtAngle) -> {
            turretCRTAngle = crtAngle;
            crtAnglePublisher.set(turretCRTAngle.in(Degrees));

        });
        enc1AnglePublisher.set(enc1Supplier.get().in(Degrees));
        enc2AnglePublisher.set(enc2Supplier.get().in(Degrees));

        Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;
        Pose2d turretBasePose = robotPose.transformBy(offsetTransform);

        Rotation2d turretRotation = Rotation2d.fromRotations(turretEncoder.getPosition());
        Rotation2d turretFieldRotation = robotPose.getRotation().plus(mountYaw).plus(turretRotation);

        // Publish a full Pose3d so mechanisms can attach at height (z)
        Pose3d turretFieldPose3d = new Pose3d(turretBasePose.getX(), turretBasePose.getY(), turretMountPose.getZ(),
                new Rotation3d(turretFieldRotation));
        turretPosePublisher.set(turretFieldPose3d);

        // Update shooter visualization length and color
        try {
            AngularVelocity vel = shooterVelocitySupplier.get();
            if (vel != null) {
                // Convert to rotations per second for a simple linear mapping
                double rps = vel.in(edu.wpi.first.units.Units.RevolutionsPerSecond);
                // Map rps to length: base 5 units + 20 * rps
                double length = 5.0 + 20.0 * Math.abs(rps);
                m_shooterLigament.setLength(length);
                // Color: blue when stopped, green when fast
                if (rps > 10.0) {
                    m_shooterLigament.setColor(new Color8Bit(Color.kLime));
                } else if (rps > 2.0) {
                    m_shooterLigament.setColor(new Color8Bit(Color.kYellow));
                } else {
                    m_shooterLigament.setColor(new Color8Bit(Color.kBlue));
                }
            }
        } catch (Exception e) {
            // Defensive: if supplier not available yet, ignore
        }
    }

    public void setAngle(Angle targetAngle) {
        turretMotor.getClosedLoopController().setSetpoint(targetAngle.in(Rotations),
                ControlType.kPosition);
        setPointPublisher.set(targetAngle.in(Degrees));
    }

    public Command SetpointCommand(Angle targetAngle) {
        return runOnce(() -> setAngle(targetAngle)).withName(getName() + " setAngle");
    };

    public Command SetMotorSpeedCommand(double speed) {
        return runOnce(() -> turretMotor.set(speed)).withName(getName() + " setSpeed");
    }

}
