package frc.robot.subsystems.TurretSubsystem;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Constants;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Turret extends SubsystemBase {
    InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

    // Motor control
    SparkMax turretMotor = new SparkMax(Constants.Turret.motorID, MotorType.kBrushless);
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
    private DoubleTopic distanceTopic = telemetryTable.getDoubleTopic("Target Distance");
    private StructTopic<Pose2d> targetPoseTopic = telemetryTable.getStructTopic("Target Angle", Pose2d.struct);
    private StructTopic<Pose2d> turretPoseTopic = telemetryTable.getStructTopic("Turret Pose", Pose2d.struct);
    private DoublePublisher crtAnglePublisher = crtAngleTopic.publish();
    private DoublePublisher enc1AnglePublisher = enc1AngleTopic.publish();
    private DoublePublisher enc2AnglePublisher = enc2AngleTopic.publish();
    private DoubleTopic setPointTopic = telemetryTable.getDoubleTopic("setPoint");
    private DoublePublisher setPointPublisher = setPointTopic.publish();
    private DoublePublisher distancePublisher = distanceTopic.publish();
    private StructPublisher<Pose2d> targetPosePublisher = targetPoseTopic.publish();
    private StructPublisher<Pose2d> turretPosePublisher = turretPoseTopic.publish();
    private StructPublisher<Pose2d> targetRelativePosePub = telemetryTable.getStructTopic("targetRelativePose", Pose2d.struct).publish();

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
            .withAbsoluteEncoderInversions(false, false);

    // you can inspect:
    // easyCrt.getUniqueCoverage(); // Optional<Angle> coverage from prime counts
    // and common scale
    // easyCrt.coverageSatisfiesRange(); // Does coverage exceed maxMechanismAngle?
    // easyCrt.getRecommendedCrtGearPair(); // Suggested pair within constraints

    // Create the solver (initialized in constructor so we can swap to a sim-only
    // supplier there)
    EasyCRT easyCrtSolver;
    boolean solveCRTperiodic = false;

    // Simulation objects
    private final SingleJointedArmSim m_turretSim;
    private final SparkMaxSim m_turretMotorSim;
    // Visualization
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_turretLigament;

    public Turret() {
        // Setup visualization
        m_mech2d = new Mechanism2d(60, 60);
        m_root = m_mech2d.getRoot("TurretRoot", 30, 30);
        m_turretLigament = m_root.append(new MechanismLigament2d("TurretArm", 30,
                0));
        SmartDashboard.putData("Turret Sim", m_mech2d);

        // Setup simulation objects
        // Use a single NEO motor model and a 30:1 gearing (motor:mech)
        final DCMotor turretGearbox = DCMotor.getNEO(1);
        final double gearingRatio = 30.0;
        // Estimate a small moment of inertia: radius 0.2 m, mass 5 kg (conservative)
        final double mechRadius = 0.1;
        final double mechMass = 2.0;
        final double mechMOI = SingleJointedArmSim.estimateMOI(mechRadius, mechMass);

        m_turretSim = new SingleJointedArmSim(turretGearbox, gearingRatio, mechMOI, mechRadius,
                -Math.PI * 100, Math.PI * 100, /* addGearingInertia */ false,
                Units.degreesToRadians(0), /* encoderDistPerPulse */ 1.0, 0.0);

        m_turretMotorSim = new SparkMaxSim(turretMotor, turretGearbox);

        // Create the CRT solver. In simulation we will provide simulated
        // absolute-encoder
        // suppliers which are derived from the turret simulation state so the CRT
        // solver
        // sees physically-correct encoder readings that reflect gearing differences.
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            Supplier<Angle> simEnc1Supplier = () -> {
                double mechRot = Units.radiansToRotations(m_turretSim.getAngleRads());
                double encRot = -Constants.Turret.enc1Zero.in(Rotations) + mechRot * (200.0 / 19.0);
                return Rotations.of(encRot);
            };
            Supplier<Angle> simEnc2Supplier = () -> {
                double mechRot = -Units.radiansToRotations(m_turretSim.getAngleRads());
                double encRot = Constants.Turret.enc2Zero.in(Rotations) + mechRot * (200.0 / 21.0);
                return Rotations.of(encRot);
            };

            EasyCRTConfig simCfg = new EasyCRTConfig(simEnc1Supplier, simEnc2Supplier)
                    .withCommonDriveGear(
                            /* commonRatio (mech:drive) */ 1,
                            /* driveGearTeeth */ 200,
                            /* encoder1Pinion */ 19,
                            /* encoder2Pinion */ 21)
                    .withAbsoluteEncoderOffsets(Constants.Turret.enc1Zero, Constants.Turret.enc2Zero)
                    .withMechanismRange(Rotations.of(-0.5), Rotations.of(0.5))
                    .withMatchTolerance(Rotations.of(0.0265))
                    .withAbsoluteEncoderInversions(true, true);

            easyCrtSolver = new EasyCRT(simCfg);
        } else {
            easyCrtSolver = new EasyCRT(easyCrt);
        }

    turretConfig
        .closedLoopRampRate(.25)
        .openLoopRampRate(.25)
        .smartCurrentLimit(20) // Neo550
        .idleMode(IdleMode.kBrake)
        // Invert the motor output so positive setpoints produce CCW rotation
        .inverted(true)
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
                .allowedClosedLoopError(Constants.Turret.toleranceAngle.in(Rotations),
                        ClosedLoopSlot.kSlot0).feedForward
                .sv(Constants.Turret.kS.baseUnitMagnitude(), Constants.Turret.kV.magnitude());
        turretConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(Constants.Turret.turretVelocity.in(Constants.Turret.turretVelocityUnit))
                .maxAcceleration(Constants.Turret.turretAccel.in(Constants.Turret.turretAccelerationUnit))
                .allowedProfileError(Constants.Turret.toleranceAngle.in(Rotations));
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // set turret position
        easyCrtSolver.getAngleOptional().ifPresent((angle) -> {
            turretMotor.getEncoder().setPosition(angle.in(Rotations));
        });

        // setup distance speed table
        // meters -> RPM
        table.put(0.0, 3000.0);
        table.put(3.2, 3000.0);
        table.put(3.7, 3200.0);
        table.put(4.7, 3600.0);
        table.put(5.56, 3900.0);
        table.put(6.7, 4200.0);
    }

    public Angle getAngle() {
        // Get the current angle of the turret
        return Constants.Turret.turretAngleUnit.of(turretMotor.getEncoder().getPosition());
    }

    public Pair<Angle, Distance> turretAngleDistance(Pose2d target) {
        Translation2d turretToTarget = RobotContainer.drivetrain.turretToTargetFieldRelative(target.getTranslation(),
                Constants.Turret.turretOffset);
        return Pair.of(turretToTarget.getAngle().getMeasure(),
                Meter.of(turretToTarget.getDistance(Translation2d.kZero)));
    }

    public Pose2d getGoalPose2d() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Blue) {
            if (Constants.Field.blueAlianceHubZone
                    .contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                return Constants.Field.BlueGoalPose2D;
            } else {
                if (Constants.Field.topZone.contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                    return Constants.Field.BlueSideBottom;
                }
                if (Constants.Field.bottomZone.contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                    return Constants.Field.BlueSideTop;
                }
            }
            return Constants.Field.BlueGoalPose2D;
        } else {
            if (Constants.Field.redAlianceHubZone
                    .contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                return Constants.Field.RedGoalPose2D;
            } else {
                if (Constants.Field.topZone.contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                    return Constants.Field.RedSideTop;
                }
                if (Constants.Field.bottomZone.contains(RobotContainer.drivetrain.getState().Pose.getTranslation())) {
                    return Constants.Field.RedSideBottom;
                }
            }
            return Constants.Field.RedGoalPose2D;
        }
    }

    public void simulationPeriodic() {
        // Update the simulated turret model. Use the motor applied output ([-1,1])
        // times
        // the current battery voltage as the input.

        m_turretSim.setInput(turretMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Update the SparkMax simulation (provides simulated internal encoder values)
        m_turretMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_turretSim.getVelocityRadPerSec()),
                RoboRioSim.getVInVoltage(), 0.020);

        // Step the mechanism simulation forward one robot loop (20ms)
     m_turretSim.update(0.020);

        // Update simulated battery voltage based on current draw
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(m_turretSim.getCurrentDrawAmps()));
        // Update CRT visualization with the mechanism angle
        double turretDeg = Units.radiansToDegrees(m_turretSim.getAngleRads());
        m_turretLigament.setAngle(turretDeg);

    }

    public Pose2d getTurretPose() {
        return RobotContainer.drivetrain.getState().Pose
        .plus(
            new Transform2d(
                Constants.Turret.TurretPos.toTranslation2d(), 
                new Rotation2d(Rotations.of(turretMotor.getEncoder().getPosition()))
            ));
    }

    private Transform2d turretTransform = new Transform2d(Constants.Turret.TurretPos.toTranslation2d(), Rotation2d.kZero);
    public void periodic() {
        // set the current CRT angle and publish it
        // not solving every iteration will improve loop time
        turretPosePublisher.set(getTurretPose());
        targetRelativePosePub.set(RobotContainer.drivetrain.targetToRobotRelative(getGoalPose2d(), turretTransform));
        if (solveCRTperiodic) {
            easyCrtSolver.getAngleOptional().ifPresent((crtAngle) -> {
                turretCRTAngle = crtAngle;
                crtAnglePublisher.set(turretCRTAngle.in(Degrees));
                turretMotor.getEncoder().setPosition(turretCRTAngle.in(Constants.Turret.turretAngleUnit));
            });
        }
        enc1AnglePublisher.set(enc1Supplier.get().in(Degrees));
        enc2AnglePublisher.set(enc2Supplier.get().in(Degrees));
        distancePublisher.set(turretAngleDistance(getGoalPose2d()).getSecond().baseUnitMagnitude());
    }

    public void setAngle(Angle targetAngle) {
        turretMotor.getClosedLoopController().setSetpoint(targetAngle.in(Rotations),
                ControlType.kPosition);
        setPointPublisher.set(targetAngle.in(Degrees));
    }
    
    public Command publishCRTangle() {
        return runOnce(() -> solveCRTperiodic = true);
    }

    public Command SetpointCommand(Angle targetAngle) {
        return runOnce(() -> setAngle(targetAngle)).withName(getName() + " setAngle");
    };

    public Command SetMotorSpeedCommand(double speed) {
        return runOnce(() -> turretMotor.set(speed)).withName(getName() + " setSpeed");
    }

    public Command AutoAimAndSpinCommand(Supplier<Pose2d> targetSupplier) {
        return new FunctionalCommand(() -> {
        },
                () -> {
                    Pose2d target = targetSupplier.get();
                    var Tad = turretAngleDistance(target);
                    Angle targetAngleField = Tad.getFirst();
                    double targetDistanceMeters = Tad.getSecond().in(Meters);
                    Rotation2d targetAngleRobot = RobotContainer.drivetrain.getState().Pose.getRotation()
                            .plus(new Rotation2d(targetAngleField));
                    targetPosePublisher.set(target);
                    setAngle(targetAngleRobot.getMeasure());
                    RobotContainer.shooter.setVelocitySetpoint(RPM.of(table.get(targetDistanceMeters)));
                },
                (interrupted) -> {
                    // turret will just go to it's last setpoint and stop, but shooter motor will
                    // keep going unless told otherwise
                    RobotContainer.shooter.setVelocitySetpoint(RPM.zero());
                },
                () -> false, // isFinished
                this, RobotContainer.shooter);
    }

    BooleanSupplier atTurretSetpoint = () -> turretMotor.getClosedLoopController().isAtSetpoint();
    BooleanSupplier atShooterSetpoint = () -> RobotContainer.shooter.isShooterAtSetSpeed();
    boolean shooterEnabled = true;

    public Command AutoAimMasterCommand() {

        new Trigger(() -> shooterEnabled)
                .onTrue(RobotContainer.hopper.startHopper())
                .onFalse(RobotContainer.hopper.stopHopper());
        return AutoAimAndSpinCommand(this::getGoalPose2d)
                .beforeStarting(() -> shooterEnabled = true)
                .finallyDo(() -> shooterEnabled = false);
    }

}
