package frc.robot.subsystems;

import javax.print.attribute.standard.PrinterLocation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;
import frc.robot.subsystems.Constants;
public class agitatormotor extends SubsystemBase {
    /** Creates a new ThroughBumberIntake. */

  private SparkMax hopperMotor2;
  private DCMotorSim hopperMotorSim;

  public agitatormotor() {

    hopperMotor2 = new SparkMax(Constants.Hopper.motorCanId2, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHopper2Power(double power) {
    hopperMotor2.set(power);
  }

  public Command startHopper2() {
    return run(() -> setHopper2Power(Constants.Hopper.hopper2Speed)); // Set to full power, adjust as needed
  }

   public Command stopHopper2() {
    return run(() -> setHopper2Power(0.0)); // Stop the intake
  }

  public Command invertHopper(){
    return run(() -> setHopper2Power(Constants.Hopper.invertedHopperSpeed));
  }

}



