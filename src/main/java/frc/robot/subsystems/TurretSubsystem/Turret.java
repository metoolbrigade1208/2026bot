package frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;



public class Turret {

public AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold

SparkMax turretMotor = new SparkMax(1, null); //swap to NEO motor
SparkMax enc2TurretMotor = new SparkMax(1, null); //swap to NEO motor
AbsoluteEncoder enc1 = turretMotor.getAbsoluteEncoder(); 
AbsoluteEncoder enc2 = enc2TurretMotor.getAbsoluteEncoder(); // Replace with a second encoder if available

Supplier<Angle> enc1Supplier = () -> Degrees.of(enc1.getPosition()); // Assuming getPosition returns rotations
Supplier<Angle> enc2Supplier = () -> Degrees.of(enc2.getPosition()); // Assuming getPosition returns rotations


SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig()
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));
SmartMotorController motor = new SparkWrapper(turretMotor,
                                                             DCMotor.getNEO(1),
                                                             motorConfig);
 
PivotConfig m_config = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      //.withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin infinitely
      .withHardLimit(Degrees.of(-180), Degrees.of(180)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("PivotExample", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation



// Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.
EasyCRTConfig easyCrt =
    new EasyCRTConfig(enc1Supplier, enc2Supplier)
        .withCommonDriveGear(
            /* commonRatio (mech:drive) */ 12.0,
            /* driveGearTeeth */ 50,
            /* encoder1Pinion */ 19,
            /* encoder2Pinion */ 23)
        .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
        .withMechanismRange(Rotations.of(-1.0), Rotations.of(2.0)) // -360 deg to +720 deg
        .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
        .withAbsoluteEncoderInversions(false, false)
        .withCrtGearRecommendationConstraints(
            /* coverageMargin */ 1.2,
            /* minTeeth */ 15,
            /* maxTeeth */ 45,
            /* maxIterations */ 30);

// you can inspect:
//easyCrt.getUniqueCoverage();          // Optional<Angle> coverage from prime counts and common scale
//easyCrt.coverageSatisfiesRange();     // Does coverage exceed maxMechanismAngle?
//easyCrt.getRecommendedCrtGearPair();  // Suggested pair within constraints

// Create the solver:
EasyCRT easyCrtSolver = new EasyCRT(easyCrt);



public void periodic() {
      if (motor.getRotorVelocity().compareTo(threshold) < 0) { // Only update when the mechanism is moving slowly to ensure accurate readings
      easyCrtSolver.getAngleOptional().ifPresent(angle -> {
    // Use the angle for your application
   motor.setEncoderPosition(angle); // Set the motor's encoder position to the calculated angle
      
});}}
public Command getSetpointCommand(Angle targetAngle) {
    return new InstantCommand(() -> {
        motor.setEncoderPosition(targetAngle);
    });

}}


