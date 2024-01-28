
package frc.team1918.robot.classes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team1918.robot.Constants;
import frc.team1918.robot.utils.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Vision Subsystem handles getting and managing data from the PhotoVision system.
 * It is responsible for getting target data, selecting appropriate targets, and passing information to other subsystems.
 */
public class Vision {
	private static Vision instance;
  private LimelightHelpers.Results llresults;
  public enum Tags {
    BLUE_SOURCE_RIGHT(1),
    BLUE_SOURCE_LEFT(2),
    BLUE_AMP(6),
    BLUE_SPEAKER_CENTER(7),
    BLUE_SPEAKER_SIDE(8),
    BLUE_STAGE_LEFT(15),
    BLUE_STAGE_RIGHT(16),
    BLUE_STAGE_CENTER(14),
    RED_SOURCE_RIGHT(9),
    RED_SOURCE_LEFT(10),
    RED_AMP(5),
    RED_SPEAKER_CENTER(4),
    RED_SPEAKER_SIDE(3),
    RED_STAGE_LEFT(11),
    RED_STAGE_RIGHT(12),
    RED_STAGE_CENTER(13);
    private final int id;
    Tags(int id) { this.id = id; }
    public int ID() { return this.id; }
  }
  public enum Targets {
    SPEAKER,
    AMP,
    STAGE_LEFT,
    STAGE_RIGHT,
    STAGE_CENTER,
    SOURCE;
  }

  /**
	 * Returns the instance of the LightingSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return LightingSubsystem instance
	 */
  public static Vision getInstance() {
		if (instance == null)
			instance = new Vision();
		return instance;
	}

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
