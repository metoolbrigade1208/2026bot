// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BumperIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Constants;

/** Creates a new ThroughBumberIntake. */
public class BumberIntake extends SubsystemBase {

  /** intake Motor SparkFlex */
  private SparkMax intakeMotor;
  private double m_armKp = Constants.OverBumperIntake.kArmKp;
  private final DCMotor m_armGearbox = DCMotor.getNeoVortex(2);
  private final SparkFlex m_armMotorLeader = new SparkFlex(Constants.OverBumperIntake.armmotorLeaderCanId,
      MotorType.kBrushless);
  private final SparkFlex m_armMotorFollower = new SparkFlex(Constants.OverBumperIntake.armmotorFollowerCanId,
      MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_armMotorLeader.getClosedLoopController();

  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_armGearbox,
      Constants.OverBumperIntake.kArmReduction,
      SingleJointedArmSim.estimateMOI(Constants.OverBumperIntake.kArmLength,
          Constants.OverBumperIntake.kArmMass),
      Constants.OverBumperIntake.kArmLength, Constants.OverBumperIntake.kMinAngleRads,
      Constants.OverBumperIntake.kMaxAngleRads, true, Units.degreesToRadians(90),
      Constants.OverBumperIntake.kArmEncoderDistPerPulse, 0.0 // Add noise with a std-dev of 1
                                                              // tick
  );
  private final RelativeEncoder m_encoder = m_armMotorLeader.getEncoder();
  private final SparkRelativeEncoderSim m_encoderSim = new SparkRelativeEncoderSim(m_armMotorLeader);
  private final SparkFlexSim m_armMotorSim = new SparkFlexSim(m_armMotorLeader, m_armGearbox);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", .1, -90));
  private final MechanismLigament2d m_arm = m_armPivot
      .append(new MechanismLigament2d("Arm", Constants.OverBumperIntake.kArmLength * 3,
          Units.radiansToDegrees(m_armSim.getAngleRads()), 6, new Color8Bit(Color.kYellow)));
public void zeroPivotPosition() {
 m_encoder.setPosition(Constants.OverBumperIntake.kArmDownPosition);

}
public void maxPivotPosition() {
  m_encoder.setPosition(Constants.OverBumperIntake.kArmUpPosition);
}
  public BumberIntake() {
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    intakeMotor = new SparkMax(Constants.OverBumperIntake.motorCanId, MotorType.kBrushless);

    // Configure the arm motor
    SparkMaxConfig armMotorLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    armMotorLeaderConfig.smartCurrentLimit(50).idleMode(IdleMode.kCoast).inverted(false);
    armMotorLeaderConfig.encoder
        .positionConversionFactor(1.0/Constants.OverBumperIntake.kArmEncoderGearing)
        .velocityConversionFactor(1.0/Constants.OverBumperIntake.kArmEncoderGearing);
    armMotorLeaderConfig.closedLoop
        .pid(Constants.OverBumperIntake.kArmKp, Constants.OverBumperIntake.kArmKi,
            Constants.OverBumperIntake.kArmKd, ClosedLoopSlot.kSlot0)
          .feedForward
            .scr(Constants.OverBumperIntake.kArmKs, Constants.OverBumperIntake.kArmCos, 1);
    armMotorLeaderConfig.softLimit
      .forwardSoftLimit(Constants.OverBumperIntake.kArmUpPosition)
      .reverseSoftLimit(Constants.OverBumperIntake.kArmDownPosition)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);
      
    armMotorLeaderConfig.closedLoop.maxMotion
        .maxAcceleration(Constants.OverBumperIntake.kArmMaxAcceleration)
        .cruiseVelocity(Constants.OverBumperIntake.kArmMaxSpeed)
        .allowedProfileError(Constants.OverBumperIntake.kArmMaxError);

    armMotorFollowerConfig.idleMode(IdleMode.kCoast).follow(m_armMotorLeader, true);

    m_armMotorLeader.configure(armMotorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotorFollower.configure(armMotorFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      .inverted(true)
      .closedLoop
        .pid(Constants.OverBumperIntake.kIntakeKp, 0,0)
        .feedForward
          .kS(0)
          .kV(1.0 / 565);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    Preferences.initDouble(Constants.OverBumperIntake.kArmPKey, m_armKp);
    m_armMotorLeader.getEncoder().setPosition(Constants.OverBumperIntake.kArmUpPosition);
    m_armMotorFollower.getEncoder().setPosition(Constants.OverBumperIntake.kArmUpPosition);
    // this.setDefaultCommand(armUpCommand());
      slamBottom().debounce(1).onTrue(runOnce(this::zeroPivotPosition).withName("Slam into Bottom Stop"));
      slamTop().debounce(1).onTrue(runOnce(this::maxPivotPosition).withName("Slam into Top Stop"));
  }

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_armMotorLeader.getAppliedOutput() * RobotController.getBatteryVoltage());

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
    
  }

  public void reachSetpoint(Double setPoint) {
    // setPoint += armUpPositionLimit;
    // System.out.print("Setting arm position: ");
    System.out.println(setPoint);
    m_controller.setSetpoint(setPoint, ControlType.kPosition);
    // m_controller2.setReference(setPoint, ControlType.kPosition);
  }

  public void stoparm() {
    m_armMotorLeader.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void TeleopInit()
  {
    m_armMotorLeader.getClosedLoopController().setSetpoint(m_encoder.getPosition(), ControlType.kPosition);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.getClosedLoopController().setSetpoint(speed, ControlType.kVelocity);
  }

  public Command startIntake() {
    return run(() -> setIntakeSpeed(Constants.OverBumperIntake.overBumperIntakeSpeed)); // Set to full power, adjust as
                                                                                        // needed
  }

  public Command stopIntake() {
    return run(() -> setIntakeSpeed(0.0)); // Stop the intake
  }

  public Command armDownCommand() {
    return runOnce(() -> this.reachSetpoint(Constants.OverBumperIntake.kArmDownPosition));
  }

  public Command armUpCommand() {
    return runOnce(() -> this.reachSetpoint(Constants.OverBumperIntake.kArmUpPosition));
  }
  public Command armPartiallyUpCommand() {
    return runOnce(() -> this.reachSetpoint(Constants.OverBumperIntake.kArmPartiallyUpPostion));
  }
  public Command armAgitateCommand() {
      return new RepeatCommand(
      armDownCommand()
      .andThen(new WaitCommand(.5))
      .andThen(armPartiallyUpCommand())
      .andThen(new WaitCommand(.5)));
    }

    public Command setArmZero() {
return runOnce(() -> m_armMotorLeader.getEncoder().setPosition(0));
    }
  BooleanSupplier excessCurrent = () -> Math.abs(m_armMotorLeader.getOutputCurrent()) > 20;
  BooleanSupplier zeroVelocity = () -> m_armMotorLeader.getEncoder().getVelocity() < 1e-4;
    BooleanSupplier negativeOutput = () -> m_armMotorLeader.getAppliedOutput() < 0;
    BooleanSupplier positiveOutput = () -> m_armMotorLeader.getAppliedOutput() > 0;
  
  public Trigger slamTop() {
    return new Trigger(excessCurrent).and(positiveOutput).and(zeroVelocity);
  }
  public Trigger slamBottom() {
    return new Trigger(excessCurrent).and(negativeOutput).and(zeroVelocity);
  }  
}
