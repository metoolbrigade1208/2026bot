// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
public class Shooter extends SubsystemBase {
  
  private AngularVelocity manualRPM = RPM.of(3000);
  // Vendor motor controller object
  private SparkFlex spark1 = new SparkFlex(52, MotorType.kBrushless);
  private SparkFlex spark2 = new SparkFlex(53, MotorType.kBrushless);

  private final AngularVelocity kVortexKv = RPM.of(565.0); // RPM/V
  private final Distance kWheelDiameter = Inches.of(4);
  private final double kShooterkV = 1.0/kVortexKv.in(RevolutionsPerSecond); //smcConfig expects V/RPS


  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(.25, 0, 0)
  .withSimClosedLoopController(25, 0, 0)
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, kShooterkV, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, kShooterkV, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  .withGearing(1.0)
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(80))
  .withFollowers(Pair.of(spark2, true));

  

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark1, DCMotor.getNeo550(1), smcConfig);

 private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel.
  .withDiameter(kWheelDiameter)
  // Mass of the flywheel.
  .withMass(Pounds.of(3))
  // Maximum speed of the shooter.
  .withUpperSoftLimit(RPM.of(6800))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  /** Creates a new ExampleSubsystem. */
  public Shooter() {}

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}

  public Command setToManualVelocity(){
    return setVelocity(manualRPM);
  }
  
  public Command bumpManualVelocity(AngularVelocity bump) {
    return runOnce( () -> manualRPM.plus(bump));
  }

  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}

  public boolean isShooterAtSetSpeed() {
    return sparkSmartMotorController.getMechanismSetpointVelocity().orElse(RPM.zero())
      .minus(sparkSmartMotorController.getMechanismVelocity())
      .lt(RPM.of(250));
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command RunShooterCommand() {
    return runOnce(
        () -> {
          // Example of setting a velocity setpoint
          shooter.setMeasurementVelocitySetpoint(FeetPerSecond.of(50.0));
        });
      
  }
public Command StopShooterCommand() {
  return runOnce(
    () -> {
      shooter.setMeasurementVelocitySetpoint(FeetPerSecond.of(0.0));
      shooter.set(0.0);
    });
}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
