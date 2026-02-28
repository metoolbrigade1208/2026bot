// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BumperIntake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Constants.Intake;

public class BumberIntake extends SubsystemBase {
  /** Creates a new ThroughBumberIntake. */
    private static BumberIntake instance;

  public static BumberIntake getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }
  private SparkMax intakeMotor;
  private DCMotor intakeMotorSim;
  private double m_armKp = Constants.OverBumperIntake.kArmKp;
  private double m_armSetpointDegrees = Constants.OverBumperIntake.kDefaultArmSetpointDegrees;
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);
  private final SparkMax m_armMotor = new SparkMax(Constants.OverBumperIntake.armmotorCanId, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_armMotor.getClosedLoopController();
  
  
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_armGearbox,
      Constants.OverBumperIntake.kArmReduction,
      SingleJointedArmSim.estimateMOI(Constants.OverBumperIntake.kArmLength,
          Constants.OverBumperIntake.kArmMass),
      Constants.OverBumperIntake.kArmLength, Constants.OverBumperIntake.kMinAngleRads,
      Constants.OverBumperIntake.kMaxAngleRads, true, Units.degreesToRadians(90),
      Constants.OverBumperIntake.kArmEncoderDistPerPulse, 0.0 // Add noise with a std-dev of 1
                                                             // tick
  );
  private final SparkAbsoluteEncoder m_encoder = m_armMotor.getAbsoluteEncoder();
  private final SparkAbsoluteEncoderSim m_encoderSim = new SparkAbsoluteEncoderSim(m_armMotor);
  private final SparkMaxSim m_armMotorSim = new SparkMaxSim(m_armMotor, m_armGearbox);

  private double armUpPositionLimit = m_encoder.getPosition();

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", .1, -90));
  private final MechanismLigament2d m_arm = m_armPivot
      .append(new MechanismLigament2d("Arm", Constants.OverBumperIntake.kArmLength * 3,
          Units.radiansToDegrees(m_armSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));


  public BumberIntake() {
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    intakeMotor = new SparkMax(Constants.OverBumperIntake.motorCanId, MotorType.kBrushless);

    
    // Configure the arm motor
    SparkMaxConfig armMotorLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorFollowerConfig = new SparkMaxConfig();
    armMotorLeaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(true);
    armMotorLeaderConfig.absoluteEncoder
        .positionConversionFactor(Constants.OverBumperIntake.kArmEncoderGearing);
    armMotorLeaderConfig.closedLoop
        .pid(Constants.OverBumperIntake.kArmKp, Constants.OverBumperIntake.kArmKi,
            Constants.OverBumperIntake.kArmKd, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(true)
        .positionWrappingInputRange(0, .8888);
    armMotorLeaderConfig.closedLoop.maxMotion
        .maxAcceleration(Constants.OverBumperIntake.kArmMaxAcceleration)
        .maxVelocity(Constants.OverBumperIntake.kArmMaxSpeed)
        .allowedClosedLoopError(Constants.OverBumperIntake.kArmMaxError);

          armMotorFollowerConfig.idleMode(IdleMode.kCoast);

          Preferences.initDouble(Constants.OverBumperIntake.kArmPositionKey, m_armSetpointDegrees);
    Preferences.initDouble(Constants.OverBumperIntake.kArmPKey, m_armKp);
    if (instance != null) {
      throw new IllegalStateException("Cannot create new instance of singleton class");
    }
    instance = this;
    // this.setDefaultCommand(armUpCommand());
  }
 public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    m_armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), 0.020);
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setPosition(Units.radiansToRotations(m_armSim.getAngleRads()));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }
   public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(Constants.OverBumperIntake.kArmPositionKey, m_armSetpointDegrees);
  }
    public void reachSetpoint(Double setPoint) {
    // setPoint += armUpPositionLimit;
    // System.out.print("Setting arm position: ");
    System.out.println(setPoint);
    m_controller.setReference(setPoint, ControlType.kPosition);
    // m_controller2.setReference(setPoint, ControlType.kPosition);
  }
   public void stoparm() {
    m_armMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public Command startIntake() {
    return run(() -> setIntakePower(Constants.OverBumperIntake.overBumperIntakeSpeed)); // Set to full power, adjust as needed
  }

  public Command stopIntake() {
    return run(() -> setIntakePower(0.0)); // Stop the intake
  }
  public Command armDownCommand() {
    return runOnce(() -> this.reachSetpoint(Constants.OverBumperIntake.kArmDownPosition));
  }

  public Command armUpCommand() {
    return runOnce(() -> this.reachSetpoint(Constants.OverBumperIntake.kArmUpPosition));
  }

  public Command armOuttakeCommand() {
    return runOnce(() -> this.reachSetpoint(0.05));
  }


}
