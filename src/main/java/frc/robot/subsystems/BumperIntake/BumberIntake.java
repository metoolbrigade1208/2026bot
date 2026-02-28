// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BumperIntake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants;

public class BumberIntake extends SubsystemBase {
  /** Creates a new ThroughBumberIntake. */
  private SparkMax intakeMotor;
  
  private DCMotor intakeMotorSim;
  


  public BumberIntake() {
    intakeMotor = new SparkMax(Constants.OverBumperIntake.motorCanId, MotorType.kBrushless);

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


}
