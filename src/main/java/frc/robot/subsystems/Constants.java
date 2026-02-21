package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Function;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import swervelib.math.Matter;

public class Constants {

 
public class BumperIntake{
public static final int kArmMotorPort = 62;
    // public static final int kArmMotor2Port = 61;
    public static final int kIntakeMotorPort = 60;
    public static final int kIRsensorport = 3;
    public static final int kArmUpLimitPort = 4;

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    public static final double kArmReduction = 64.0;
    public static final double kArmEncoderGearing = (4.0 / 1.5) / 3.0; // ratio of encoder position
                                                                       // to arm
    // position
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-15);
    public static final double kMaxAngleRads = Units.degreesToRadians(100);
    public static final double kArmKp = 1.0;
    public static final double kArmKi = 0.0;
    public static final double kArmKd = 0.05;
    public static final double kArmKs = 0;
    public static final double kArmKg = 0;
    public static final double kArmKv = 0;
    public static final double kArmKa = 0;
    public static final double kArmDownPosition = Units.degreesToRotations(90);
    public static final double kArmUpPosition = 0;
    public static final double kArmMaxSpeed = 100;
    public static final double kArmMaxAcceleration = 150;
    public static final double kArmMaxError = Units.degreesToRotations(1);

    public static final double kBumperIntakeRunSpeed = .5;

    public static final double kBumperIntakeKp = 1.0;
    public static final double kBumperIntakeKi = 0.0;
    public static final double kBumperIntakeKd = 0.01;
    public static final double kBumperIntakeKv = 473;
    public static final double kBumperIntakeMaxSpeed = 0.5;
    public static final double kBumperIntakeMaxAcceleration = 0.5;
    public static final double kBumperIntakeMaxError = 0.5;

    public static final double kArmPositionConversionFactor = 2 * Math.PI;


}}
