// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.PrinterLocation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

public class Hopper extends SubsystemBase {
  /** Creates a new ThroughBumberIntake. */
private SparkMaxConfig hopperConfig = new SparkMaxConfig(); 
private static Hopper instance;
public static Hopper getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Instance not created yet");
    }
    return instance;
  }

  private SparkMax hopperMotor;

  private DCMotorSim hopperMotorSim;

  public Hopper() {
    hopperConfig.smartCurrentLimit(20).closedLoop
                .pid(0.1, 0,0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .feedForward.kV(1.0/473);
    hopperMotor = new SparkMax(Constants.Hopper.motorCanId, MotorType.kBrushless);
    hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHopperPower(double power) {
    hopperMotor.getClosedLoopController().setSetpoint(power, ControlType.kVelocity);
  }

  public Command startHopper() {
    return run(() -> setHopperPower(Constants.Hopper.hopperSpeed)); // Set to full power, adjust as needed
  }

  public Command stopHopper() {
    return run(() -> setHopperPower(0.0)); // Stop the intake
  }
}