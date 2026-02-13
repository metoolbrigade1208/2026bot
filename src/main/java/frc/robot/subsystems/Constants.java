// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Function;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import swervelib.math.Matter;

public class Constants {
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
}
/* public static class TurretConstants {
    public static final int motorCanId = 0; //change this twin
    public static final double motorReduction = 15.0;
    public static final int currentLimit = 40;
  }

 public static class QuestNavConstants {
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d( /*TODO: Put your x, y, z, yaw, pitch, and roll offsets here! );
  } */
