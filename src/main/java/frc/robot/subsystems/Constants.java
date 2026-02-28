// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Function;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.RobotConfig;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import swervelib.math.Matter;

public class Constants {
    public static final Optional<RobotConfig> config =
      loadConfig(Filesystem.getDeployDirectory().toPath().resolve("config.json").toString());

  private static double getConfigValue(Function<RobotConfig, Double> mapper, double defaultValue) {
    return config.flatMap(c -> Optional.ofNullable(mapper.apply(c))).orElse(defaultValue);
  }
  public static class RobotConfig {
    public double armKp = 1.0;
    public double armKi = 0.0;
    public double armKd = 0.01;
  }
  public static final double kTrackWidth = Units.inchesToMeters(20.0);
  public static final double kWheelBase = Units.inchesToMeters(20.0);
  public static final double kWheelDiameter = Units.inchesToMeters(4.0);
  public static final double kWheelCircumference = kWheelDiameter * Math.PI;
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularVelocity = Math.PI; // radians per second
  
  public class Intake {
    public static final int motorCanId = 50; 
    public static final double motorReduction = 15.0;
    public static final int currentLimit = 40;
    public static final double intakeSpeed = 0.9; // Adjust as needed
  }
    static Optional<RobotConfig> loadConfig(String path) {
    ObjectMapper objectMapper = new ObjectMapper();
    try {
      return Optional.of(objectMapper.readValue(new File(path), RobotConfig.class));
    } catch (IOException e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }
  public class OverBumperIntake {
    public static final int motorCanId = 57; 
     public static final int armmotorCanId = 8;
    public static final double OverBumperCurrentLimitmotorReduction = 15.0;
    public static final int overbumperCurrentLimit = 40;
    public static final double overBumperIntakeSpeed = 0.9;
    public static final double kArmKp = getConfigValue(c -> c.armKp, 1.0);
    public static final double kArmKi = getConfigValue(c -> c.armKi, 0);
    public static final double kArmKd = getConfigValue(c -> c.armKd, 0.01);
    public static final double kArmks = 0; 
    public static final double kArmKg = 0;
    public static final double kArmKv = 0;
    public static final double kArmKa = 0;
    public static final double kArmDownPosition = Units.degreesToRotations(90);
    public static final double kArmUpPosition = 0;
    public static final double kArmMaxSpeed = 100;
    public static final double kArmMaxAcceleration = 150;
    public static final double kArmMaxError = Units.degreesToRotations(1);
    public static final double kArmPositionConversionFactor = 2 * Math.PI; // Adjust as needed
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-15);
    public static final double kMaxAngleRads = Units.degreesToRadians(100);
    public static final double kArmReduction = 64.0;
    public static final double kArmEncoderGearing = (4.0 / 1.5) / 3.0;
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
     public static final String kArmPositionKey = "ArmPosition";
     public static final String kArmPKey = "ArmP";
}
}
/* public static class TurretConstants {
    public static final int motorCanpId = 0; //change this twin
    public static final double motorReduction = 15.0;
    public static final int currentLimit = 40;
  }

 public static class QuestNavConstants {
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll offsets here! );
  } */
