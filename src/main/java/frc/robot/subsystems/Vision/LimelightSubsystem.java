// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
  private Limelight limelight;
  private LimelightPoseEstimator poseEstimator;

  Matrix<N3, N1> kLimelightSD = VecBuilder.fill(0.1, 0.1, 0.1);

  NetworkTableEntry stddevEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("stddevs");
  double[] stdDevArray = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.1, 0.1, 0.0, 0.0, 0.0, 0.1 };

  /** Creates a new Limelight. */
  public LimelightSubsystem() {
    limelight = new Limelight("limelight");
    // Set the limelight to use Pipeline LED control, with the Camera offset of 0,
    // and save.
    limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(new Pose3d(Inches.of(0), Inches.of(0), Inches.of(20),
            new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(180))))
        .save();
    useAprilTags();
    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
    botUninitialized().onTrue(useCameraCommand()
      .beforeStarting(runOnce(() -> isUninitialized = false)
          .alongWith(RobotContainer.QNS.enableQuestNavCommand())));
  }


  @Override
  public void periodic() {
if (currentPipe == PipelineId.aprilTag) {
    // This method will be called once per scheduler run
    // Required for megatag2 in periodic() function before fetching pose.
    Pigeon2 gyro = RobotContainer.drivetrain.getPigeon2();
    limelight.getSettings()
        .withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
            new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
                DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
                DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble()))))
        .save();

    stdDevArray = stddevEntry.getDoubleArray(stdDevArray);
    kLimelightSD = VecBuilder.fill(stdDevArray[6], stdDevArray[7], stdDevArray[11]);

    // Get MegaTag2 pose
    Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
    // If the pose is present
    visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      if(poseEstimate.tagCount > 0){
      // Add it to the pose estimator.
      RobotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds,
          kLimelightSD);
        }
    });}

  }

  enum PipelineId {
    aprilTag(0),
    camera(1);

    private final int value;

    private PipelineId(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  private boolean isUninitialized = true;
  private Pose2d previousPose = Pose2d.kZero;

  private BooleanSupplier isPoseNotNull = () -> RobotContainer.drivetrain.getState().Pose.getTranslation()
      .getSquaredNorm() > 0.1;

  private double poseVelocity() {
    Pose2d curPose = RobotContainer.drivetrain.getState().Pose;
    var diff = curPose.minus(previousPose);

    previousPose = curPose;
    return diff.getTranslation().getSquaredNorm();
  }

  private BooleanSupplier isPoseVelocityLow = () -> poseVelocity() < 0.5;
  private PipelineId currentPipe;

  public Trigger botUninitialized() {
    return new Trigger(() -> isUninitialized)
      .and(isPoseNotNull)
      .and(isPoseVelocityLow)
      .and(RobotContainer.QNS::isTracking);
  }

  public Command useAprilTagsCommand() {
    return runOnce(this::useAprilTags).ignoringDisable(true).withName("Limelight AprilTag");
  }

  public void useAprilTags() {
    usePipeline(PipelineId.aprilTag);
  }

  private void usePipeline(PipelineId pipeline) {
    limelight.getSettings().withPipelineIndex(pipeline.value).save();
    currentPipe = pipeline;
  }

  public Command useCameraCommand() {
    return runOnce(this::useCamera).ignoringDisable(true).withName("Limelight Camera");
  }

  public void useCamera() {
    usePipeline(PipelineId.camera);
  }

  public Command reInitializeCommand () {
    return runOnce(() -> {isUninitialized = true;})
      .alongWith(RobotContainer.QNS.disableQuestNavCommand())
      .withName("reinitialize bot Pose");
  }

  public Command initializedCommand () {
    return useCameraCommand().ignoringDisable(true)
            .alongWith(RobotContainer.QNS.enableQuestNavCommand());
  }

}
