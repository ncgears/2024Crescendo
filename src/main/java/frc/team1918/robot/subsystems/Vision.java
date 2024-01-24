
package frc.team1918.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team1918.robot.Constants;
import frc.team1918.robot.utils.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Vision Subsystem handles getting and managing data from the PhotoVision system.
 * It is responsible for getting target data, selecting appropriate targets, and passing information to other subsystems.
 */
public class Vision {
  private LimelightHelpers.Results llresults;

  public Vision() {
  }

  public void updateDashboard() {
    // Dashboard.Vision.setVisionRinglight(llresults.targetingResults.);
  }

  public void updateResults() {
    llresults = LimelightHelpers.getLatestResults(Constants.Vision.limelightName).targetingResults;
  }

  /**
   * This enables or disables the ring light
   * @param enabled - true to turn on light, false to turn it off
   */
  public void setlight(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceOn(Constants.Vision.limelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.limelightName);
    }
  }

  public Pose2d getPose() {
    return LimelightHelpers.getBotPose2d(Constants.Vision.limelightName);
  }

  public Pose2d getAlliancePose(Alliance alliance) {
    if (alliance == Alliance.Blue) return llresults.getBotPose2d_wpiBlue();
    if (alliance == Alliance.Red) return llresults.getBotPose2d_wpiRed();
    return llresults.getBotPose2d();
  }

  public double getTimestamp() {
    return llresults.timestamp_LIMELIGHT_publish;
  }

  public double getDistance() {
    return 0.0; //llresults.targetingResults.targets_Fiducials[0].distanceMeters;
    //TODO: Figure out what we should do here, this is used to decide the validity of the targets? See DriveSubsystem
  }

  public double getNumTags() {
    return (int) llresults.targets_Fiducials.length;
  }

}
