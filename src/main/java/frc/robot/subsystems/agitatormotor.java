package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class agitatormotor extends SubsystemBase {
  /** Creates a new ThroughBumberIntake. */

  private SparkMax hopperMotor2;
  private DCMotorSim hopperMotorSim;
  private SparkMaxConfig agitatorConfig = new SparkMaxConfig();

  public agitatormotor() {

    hopperMotor2 = new SparkMax(Constants.Hopper.motorCanId2, MotorType.kBrushless);
    agitatorConfig.smartCurrentLimit(20).closedLoop
        .pid(0.1, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder).feedForward.kV(1.0 / 917);
    hopperMotor2.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    new Trigger(() -> { return hopperMotor2.getOutputCurrent() >= 15.0; })
      .and(() -> { return Math.abs(hopperMotor2.getEncoder().getVelocity()) <= 10;})
        .onTrue(invertHopper()
        .andThen(new WaitCommand(.5))
        .andThen(startHopper2()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHopper2Power(double power) {
    hopperMotor2.getClosedLoopController().setSetpoint(power, ControlType.kVelocity);
  }

  public Command startHopper2() {
    return run(() -> setHopper2Power(Constants.Hopper.hopper2Speed)); // Set to full power, adjust as needed
  }

  public Command stopHopper2() {
    return run(() -> setHopper2Power(0.0)); // Stop the intake
  }

  public Command invertHopper() {
    return run(() -> setHopper2Power(Constants.Hopper.invertedHopperSpeed));
  }

}
