package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private final double QUEST_NAV_HEIGHT = 10.0;
  private final double QUEST_NAV_FORWARD_CENTER_OFFSET = -16;
  private final double QUEST_NAV_LEFT_CENTER_OFFSET = +10;
  private final double QUEST_NAV_RADIAN_YAW_OFFSET = Math.atan2(-7.5, 6.5);
  private boolean enabled = false;

  public boolean isTracking() {
    return questNav.isTracking();
  }

  public boolean isEnabled() {
    return enabled;
  }

  // private Transform2d QUEST_TO_ROBOT2D = new
  // Transform2d(Units.inchesToMeters(15.0), Units.inchesToMeters(0), new
  // Rotation2d(0));
  private Transform3d QUEST_TO_ROBOT = new Transform3d(
      Inches.of(QUEST_NAV_FORWARD_CENTER_OFFSET), 
      Inches.of(QUEST_NAV_LEFT_CENTER_OFFSET),
      Inches.of(QUEST_NAV_HEIGHT),
      new Rotation3d(Units.degreesToRadians(0), 0, QUEST_NAV_RADIAN_YAW_OFFSET));
  private CommandSwerveDrivetrain swerveSubsystem;
  Pose3d roboPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

  // Define the publisher as a class-level variable to keep it active
  StructPublisher<Pose3d> posePub = NetworkTableInstance.getDefault()
      .getStructTopic("QuestNavPose3d", Pose3d.struct)
      .publish();
  private Trigger isTracking = new Trigger(this::isTracking).debounce(1);
  /** Creates a new QuestNav. */
  public QuestNavSubsystem(CommandSwerveDrivetrain swerveSubsystem) {
    
    // Warn when battery reaches 20% or below
    questNav.onLowBattery(20, level ->
        DriverStation.reportWarning("Quest battery low: " + level + "%", false)
    );

    this.swerveSubsystem = swerveSubsystem;
  }

  public void updateVisionMeasurement() {

    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

    SmartDashboard.putBoolean("QuestNav.isConnected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav.isTracking", questNav.isTracking());
    SmartDashboard.putNumber("QuestNav.batteryPercent", getQuestNavPower());

    if (questNav.isConnected() && questNav.isTracking()) {

      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

      // Loop over the pose data frames nd send them to the pose estimatior

      for (PoseFrame questFrame : questFrames) {
        // Get the Pose of the Quest
        Pose3d questPose = questFrame.questPose3d();

        // get the timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get the robot pose
        Pose3d robotPose = questPose.transformBy(QUEST_TO_ROBOT.inverse());

        // Add the mesaurement to the pose Estimator
        if (swerveSubsystem != null) {
          swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
        roboPose = robotPose;

      }
    }
  }

  public boolean isQuestnavSufficientlyCharged() {
    if (getQuestNavPower() < 15)
      return false;
    else {
      return true;
    }
  }

  public void setQuestNavPose(Pose3d pose) {
    questNav.setPose(pose.transformBy(QUEST_TO_ROBOT));
  }

  public void setQuestNavPose(Pose2d pose) {
    Pose3d pose3d = new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(pose.getRotation()));
    questNav.setPose(pose3d.transformBy(QUEST_TO_ROBOT));
  }

  public Pose3d getNavQuestPose3d() {
    return roboPose;
  }

  public boolean getQuestNavConnected() {
    return questNav.isConnected();
  }

  public boolean getQuestNavIsTracking() {
    return questNav.isTracking();
  }

  public int getQuestNavPower() {
    return questNav.getBatteryPercent().getAsInt();
  }

  public void zeroQuestNavPose() {
    setQuestNavPose(new Pose3d(Units.inchesToMeters(343), Units.inchesToMeters(14.5), 0,
        new Rotation3d(Units.degreesToRadians(0), 0, 0)));
  }

  public Command zeroQuestNavPoseCommand() {
    return runOnce(() -> zeroQuestNavPose()).withName("zero QuestNav Pose");
  }

  /**
   * Enables the QuestNav to start tracking. Pushes current robot pose to the QuestNav to initialize.
   * @return
   */
  public Command enableQuestNavCommand() {
    return runOnce(() -> {
      enabled = true;
      setQuestNavPose(swerveSubsystem.getState().Pose);
    }).withName("QuestNav enabled");
  }

public Command disableQuestNavCommand() {
  return runOnce(()-> {
    enabled = false;
  }).withName("QuestNav disabled");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    questNav.commandPeriodic();
    if (enabled) {
      updateVisionMeasurement();
      posePub.set(roboPose);
    }
  }
}