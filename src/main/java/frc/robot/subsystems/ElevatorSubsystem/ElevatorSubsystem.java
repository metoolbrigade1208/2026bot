package frc.robot.subsystems.ElevatorSubsystem;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase
{
  // TODO: Add detailed comments explaining the example, similar to the ExponentiallyProfiledArmSubsystem
  private Distance winchCircumference = Inches.of(1.6); 
  private AngularVelocity kNeoKv = RPM.of(473);
  private AngularVelocity kPostGearbox = kNeoKv.div(144);
  private double kV = (1.0 / kPostGearbox.in(RevolutionsPerSecond))/winchCircumference.in(Meters); // Volts per (Meter per Second)

  private final SparkMax elevatorMotor = new SparkMax(50, SparkLowLevel.MotorType.kBrushless);
  private final DigitalInput m_upLimitIrSensor = new DigitalInput(Constants.Climber.kIRsensorport);

  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(winchCircumference)
      .withClosedLoopController(60, 0, 0, MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5))
      .withSoftLimit(Inches.zero(), Inches.of(7)) 
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(9, 4, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ElevatorFeedforward(0, 0, kV, 10)) 
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController motor = new SparkWrapper(elevatorMotor, DCMotor.getNEO(1), motorConfig);

  private final MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private ElevatorConfig m_config = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Inches.zero(), Inches.of(7.5)) 
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(m_robotToMechanism)
      .withMass(Pounds.of(1));
  private final Elevator m_elevator = new Elevator(m_config);

  public ElevatorSubsystem()
  {

  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

  
  //Sets the elevator motor to 0 volts.
  public Command stopElevator()
  {
    return m_elevator.setVoltage(Volts.of(0));
  }

  //Runs the elevator to climb on the bar without crushing the bot
  boolean isClimbing = false;
  public Command elevatorClimb()
  {    
    new Trigger(() -> isClimbing).and( () -> !(m_upLimitIrSensor.get()) )
      .onTrue(stopElevator()
      .finallyDo(() -> isClimbing = false));
    return m_elevator.setHeight(Inches.of(2)) 
      .beforeStarting(() -> isClimbing = true); 
  }

  public Command sysId()
  {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

}