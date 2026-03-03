package frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {

    // Constants
    private static AngleUnit turretAngleUnit = Rotations;
    private static AngularVelocityUnit turretVelocityUnit = turretAngleUnit.per(Second);
    private static AngularAccelerationUnit turretAccelerationUnit = turretVelocityUnit.per(Second);
    private static final AngularAcceleration turretAccel = DegreesPerSecondPerSecond.of(900);
    private static final AngularVelocity turretVelocity = DegreesPerSecond.of(300);
    private static final Angle fwdLimit = Degrees.of(170);
    private static final Angle revLimit = Degrees.of(-170);
    private static final Angle gearing = Rotations.of(1.0).div(30); // sparkMax native unit is rotations
    private static final AngularVelocity gearSpeed = gearing.per(Second);
    private static final int motorID = 55;
    private static final int enc1Id = 0; // DIO port of encoder 1
    private static final int enc2Id = 1; // DIO port of encoder 2
    private static final Angle enc1Zero = Rotations.of(0.0); // actual zero location of encoder 1
    private static final Angle enc2Zero = Rotations.of(0.0); // actual zero location of encoder 2
    private static final double kP = 10; // output per angle difference (V/rotation)
    private static final double kD = 0; // output per angle difference derivative (V/rps)
    private static final Voltage kS = Volts.of(0.5);
    private static final Voltage kV = Volts.of(1); // really Volts/rps, but dimensions get wonky with doing all that.

    static public AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold
    static public Angle toleranceAngle = Degrees.of(0.1); // Set a threshold

    // Motor control
    SparkMax turretMotor = new SparkMax(motorID, MotorType.kBrushless);
    SparkMaxConfig turretConfig = new SparkMaxConfig();

    // Easy CRT
    DutyCycleEncoder enc1Encoder = new DutyCycleEncoder(enc1Id);
    DutyCycleEncoder enc2Encoder = new DutyCycleEncoder(enc2Id);
    Supplier<Angle> enc1Supplier = () -> turretAngleUnit.of(enc1Encoder.get());
    Supplier<Angle> enc2Supplier = () -> turretAngleUnit.of(enc2Encoder.get());

    Angle turretCRTAngle;

    /**
     * telemetry table.
     */
    private NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry\\Turret");
    private DoubleTopic crtAngleTopic = telemetryTable.getDoubleTopic("CRTAngle");
    private DoublePublisher crtAnglePublisher = crtAngleTopic.publish();
    private DoubleTopic setPointTopic = telemetryTable.getDoubleTopic("setPoint");
    private DoublePublisher setPointPublisher = setPointTopic.publish();

    // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T
    // and 23T pinions.
    EasyCRTConfig easyCrt = new EasyCRTConfig(enc1Supplier, enc2Supplier)
            .withCommonDriveGear(
                    /* commonRatio (mech:drive) */ 1,
                    /* driveGearTeeth */ 200,
                    /* encoder1Pinion */ 19,
                    /* encoder2Pinion */ 23)
            .withAbsoluteEncoderOffsets(enc1Zero, enc2Zero) // set after mechanical zero
            .withMechanismRange(Rotations.of(-0.5), Rotations.of(0.5)) // -180 deg to +180 deg
            .withMatchTolerance(Rotations.of(0.0265)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(false, false);

    // you can inspect:
    // easyCrt.getUniqueCoverage(); // Optional<Angle> coverage from prime counts
    // and common scale
    // easyCrt.coverageSatisfiesRange(); // Does coverage exceed maxMechanismAngle?
    // easyCrt.getRecommendedCrtGearPair(); // Suggested pair within constraints

    // Create the solver:
    EasyCRT easyCrtSolver = new EasyCRT(easyCrt);

    public Turret() {
        turretConfig
                .closedLoopRampRate(.25)
                .openLoopRampRate(.25)
                .smartCurrentLimit(20) // Neo550
                .idleMode(IdleMode.kBrake).encoder
                .positionConversionFactor(gearing.in(turretAngleUnit))
                .velocityConversionFactor(gearSpeed.in(turretAngleUnit.per(Minute))); // default is RPM, not RPS
        turretConfig.softLimit
                .forwardSoftLimit(fwdLimit.in(turretAngleUnit))
                .reverseSoftLimit(revLimit.in(turretAngleUnit))
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        turretConfig.closedLoop
                .pid(kP, 0, kD)
                .allowedClosedLoopError(toleranceAngle.in(Rotations), ClosedLoopSlot.kSlot0).feedForward
                .sv(kS.baseUnitMagnitude(), kV.magnitude());
        turretConfig.closedLoop.maxMotion
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                .cruiseVelocity(turretVelocity.in(turretVelocityUnit))
                .maxAcceleration(turretAccel.in(turretAccelerationUnit))
                .allowedProfileError(toleranceAngle.in(Rotations));
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Angle getAngle() {
        return turretAngleUnit.of(turretMotor.getEncoder().getPosition()); // Get the current angle of the turret
    }

    public void simulationPeriodic() {
        // Simulate the turret's behavior here if needed
    }

    public void periodic() {
        // set the current CRT angle and publish it
        // TODO: use it to set the current turret readout angle
        easyCrtSolver.getAngleOptional().ifPresent((crtAngle) -> {
            turretCRTAngle = crtAngle;
            crtAnglePublisher.set(turretCRTAngle.in(Degrees));
        });

    }

    public void setAngle(Angle targetAngle) {
        turretMotor.getClosedLoopController().setSetpoint(targetAngle.in(Rotations),
                ControlType.kMAXMotionPositionControl);
        setPointPublisher.set(targetAngle.in(Degrees));
    }

    public Command SetpointCommand(Angle targetAngle) {
        return runOnce(() -> setAngle(targetAngle)).withName(getName() + " setAngle");
    };

    public Command SetMotorSpeedCommand(double speed) {
        return runOnce(() -> turretMotor.set(speed)).withName(getName() + " setSpeed");
    }

}
