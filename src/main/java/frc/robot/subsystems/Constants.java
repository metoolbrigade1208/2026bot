// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Function;

import org.ironmaple.simulation.Goal.PositionChecker;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.RobotConfig;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.TurretSubsystem.Turret;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotations;

import swervelib.math.Matter;

public class Constants {
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

  public class Hopper {
    public static final int motorCanId = 8; 
    public static final int motorCanId2 = 9;
    public static final double motorReduction = 15.0;
    public static final int currentLimit = 40;
    public static final double hopperSpeed = 0.9;
    public static final double hopper2Speed = -0.9; // Adjust as needed
    public static final double invertedHopperSpeed = 0.9;
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
    public static final int motorCanId = 60; 
    public static final int armmotorFollowerCanId = 58;
    public static final int armmotorLeaderCanId = 57;
    public static final double OverBumperCurrentLimitmotorReduction = 15.0;
    public static final int overbumperCurrentLimit = 40;
    public static final double overBumperIntakeSpeed = 2300; //RPM
    public static final double kArmKp = 1.0;
    public static final double kArmKi = 0;
    public static final double kArmKd =  0.01;
    public static final double kArmEncoderGearing = 23.0; 
    public static final double kArmKs = 0;
    public static final double kArmCos = 0;
    public static final double kArmKv = kArmEncoderGearing / 565.0;
    public static final double kArmDownPosition = 0;
    public static final double kArmUpPosition = 0.35; //TODO: find correct angle
    public static final double kArmMaxSpeed = 10;
    public static final double kArmMaxAcceleration = 15;
    public static final double kArmMaxError = Units.degreesToRotations(1);
    public static final double kArmPositionConversionFactor = 2 * Math.PI; // Adjust as needed
    public static final double kDefaultArmKp = 50.0;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-15);
    public static final double kMaxAngleRads = Units.degreesToRadians(100);
    public static final double kArmReduction = 64.0;
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
     public static final String kArmPositionKey = "ArmPosition";
     public static final String kArmPKey = "ArmP";
    public static final double kIntakeKp = 0.0; 
}

public static class TurretConstants {
    public static final int motorCanpId = 0; //change this twin
    public static final double motorReduction = 15.0;
    public static final int currentLimit = 40;
  }

  public class Turret {
    public static AngleUnit turretAngleUnit = Rotations;
    public static AngularVelocityUnit turretVelocityUnit = turretAngleUnit.per(Second);
    public static AngularAccelerationUnit turretAccelerationUnit = turretVelocityUnit.per(Second);
    public static final AngularAcceleration turretAccel = DegreesPerSecondPerSecond.of(900);
    public static final AngularVelocity turretVelocity = DegreesPerSecond.of(300);
    public static final Angle fwdLimit = Degrees.of(180);
    public static final Angle revLimit = Degrees.of(-180);
    public static final Angle gearing = Rotations.of(1.0).div(30); // sparkMax native unit is rotations
    public static final AngularVelocity gearSpeed = gearing.per(Second);
    public static final int motorID = 55;
    public static final int enc1Id = 0; // DIO port of encoder 1
    public static final int enc2Id = 1; // DIO port of encoder 2    
    public static final Angle enc1Zero = Degrees.of(-92.9); // actual zero location of encoder 1
    public static final Angle enc2Zero = Degrees.of(-137.2); // actual zero location of encoder 2
    public static final double kP = 2.5; // output per angle difference (V/rotation)
    public static final double kD = 0.25; // output per angle difference derivative (V/rps)
    public static final Voltage kS = Volts.of(0.5);
    public static final Voltage kV = Volts.of(5); // really Volts/rps, but dimensions get wonky with doing all that.
    public static final Translation2d turretOffset = new Translation2d(Inches.of(10), Inches.of(0));
    static public AngularVelocity threshold = DegreesPerSecond.of(5); // Set a threshold
    static public Angle toleranceAngle = Degrees.of(1); // Set a threshold
    public static final Translation3d TurretPos = new Translation3d(Inches.of(-8.0), Inches.of(0.0), Inches.of(20.0));

   
  }
  public class Field{
  public static final Rectangle2d blueAlianceHubZone = new Rectangle2d(Translation2d.kZero, new Translation2d(Meters.of(4.0), Meters.of(8.0)));
  public static final Rectangle2d redAlianceHubZone = new Rectangle2d(new Translation2d(Meters.of(12.50), Meters.of(0.0)), new Translation2d(Meters.of(4.0), Meters.of(8.0)));
  public static final Rectangle2d topZone = new Rectangle2d(new Translation2d(Meters.of(0.0), Meters.of(4.0)), new Translation2d(Meters.of(16.50), Meters.of(8.0)));
  public static final Rectangle2d bottomZone = new Rectangle2d(Translation2d.kZero, new Translation2d(Meters.of(16.5), Meters.of(4.0)));
  
 public static final AprilTagFieldLayout Field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
 public static final Transform2d sideTopOffset = new Transform2d(Meters.of(0.0), Meters.of(1.0), Rotation2d.kZero);
 public static final Transform2d sideBottomOffset = new Transform2d(Meters.of(0.0), Meters.of(-1.0), Rotation2d.kZero);
    public static final Pose2d RedGoalPose2D = Field.getTagPose(4).get()
        .interpolate(Field.getTagPose(10).get(), .5)
        .toPose2d();
    public static final Pose2d BlueGoalPose2D = Field.getTagPose(20).get()
        .interpolate(Field.getTagPose(26).get(), .5)
        .toPose2d();
    public static final Pose2d BlueSideTop = Field.getTagPose(26).get().toPose2d().plus(sideTopOffset);
    public static final Pose2d BlueSideBottom = Field.getTagPose(26).get().toPose2d().plus(sideBottomOffset);
    public static final Pose2d RedSideTop = Field.getTagPose(10).get().toPose2d().plus(sideTopOffset);
    public static final Pose2d RedSideBottom = Field.getTagPose(10).get().toPose2d().plus(sideBottomOffset);
  }
}
/*
 * public static class TurretConstants {
 * public static final int motorCanpId = 0; //change this twin
 * public static final double motorReduction = 15.0;
 * public static final int currentLimit = 40;
 * }
 * 
 * public static class QuestNavConstants {
 * public static final Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put
 * your x, y, z, yaw, pitch, and roll offsets here! );
 * }
 */
