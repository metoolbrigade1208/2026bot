// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight;

  /** Creates a new Limelight. */
  public LimelightSubsystem() {
    // Set the limelight to use Pipeline LED control, with the Camera offset of 0,
    // and save.
    limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(new Pose3d(Inches.of(0), Inches.of(0), Inches.of(20), 
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(0))))
        .save();
        useAprilTags();

        botUninitialized().onTrue(useCameraCommand()
          .alongWith(runOnce(()->isUninitialized = false)
          .alongWith(RobotContainer.QNS.enableQuestNavCommand())));
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // Required for megatag2 in periodic() function before fetching pose.
    Pigeon2 gyro = RobotContainer.drivetrain.getPigeon2();
    limelight.getSettings()
        .withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
            new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
                DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
                DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble()))))
        .save();

    // Get MegaTag2 pose
    Optional<PoseEstimate> visionEstimate = limelight.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    // If the pose is present
    visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      // Add it to the pose estimator.
      RobotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds,kLimelightSD);
    });

  }

  private int aprilTagPiplineId = 0;
  private int cameraPiplineId = 1;
  private boolean isUninitialized = true;
  private Pose2d previousPose = Pose2d.kZero;

  private BooleanSupplier isPoseNotNull = () -> RobotContainer.drivetrain.getState().Pose.getTranslation()
      .getSquaredNorm() > 0.1;

  private double poseVelocity() {
    Pose2d curPose = RobotContainer.drivetrain.getState().Pose;
    var diffence = curPose.minus(previousPose);

    previousPose = curPose;
    return diffence.getTranslation().getSquaredNorm();
  }

  private BooleanSupplier isPoseVelocityLow = () -> poseVelocity() < 0.1;

  public Trigger botUninitialized() {
    return new Trigger(() -> isUninitialized).and(isPoseNotNull).and(isPoseVelocityLow);
  }

  public Command useAprilTagsCommand() {
    return runOnce(this::useAprilTags).ignoringDisable(true);
  }

  public void useAprilTags() {
    limelight.getSettings().withPipelineIndex(aprilTagPiplineId).save();
  }

  public Command useCameraCommand() {
    return runOnce(this::useCamera).ignoringDisable(true);
  }

  public void useCamera() {
    limelight.getSettings().withPipelineIndex(cameraPiplineId).save();
  }

}
