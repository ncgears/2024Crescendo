package frc.team1918.robot.classes;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.team1918.robot.Constants;
import frc.team1918.robot.RobotContainer;

/**
 * The NCPose class handles getting and managing the different poses and calculations of them.
 * It is responsible for maintaining the robot poses and exposing methods to calculate items related to the poses
 */
public class NCPose {
	private static NCPose instance;
	private SwerveDrivePoseEstimator poseEstimator;

    public NCPose() {
		//Initialize the pose estimator
		poseEstimator = new SwerveDrivePoseEstimator(
			Constants.Swerve.kDriveKinematics,
			RobotContainer.gyro.getHeading(),
			RobotContainer.drive.getSwerveModulePositions(),
			new Pose2d(),
			//stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust your state estimate less.
			VecBuilder.fill(0.1, 0.1, 0.1),
			//visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
			VecBuilder.fill(0.9, 0.9, 0.9)
		);
    }

    /**
	 * Returns the instance of the class.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return instance of this class
	 */
    public static NCPose getInstance() {
		if (instance == null)
			instance = new NCPose();
		return instance;
	}
    
	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
     * Reset the estimated pose of the swerve drive on the field.
     *
	 * @param heading Heading to reset robot to (for configuring a yaw offset)
     * @param pose New robot pose.
     */
	public void resetPose(double heading, Pose2d pose) {
		poseEstimator.resetPosition(Rotation2d.fromDegrees(heading), RobotContainer.drive.getSwerveModulePositions(), pose);
	}

	/**
	 * Updates the bot pose based on feedback from the drivetrain. This should be done in every periodic loop.
	 */
	public void updatePose() {
		poseEstimator.update(
			RobotContainer.gyro.getHeading(),
			RobotContainer.drive.getSwerveModulePositions()
		);
	}

	/**
	 * Corrects the bot pose based on information from the vision system
	 */
	public void correctPoseWithVision() {
		var visionEstFront = RobotContainer.vision.getEstimatedGlobalPose("front");
		visionEstFront.ifPresent(
			est -> {
				var estPose = est.estimatedPose.toPose2d();
				// Change our trust in the measurement based on the tags we can see
				var estStdDevs = RobotContainer.vision.getEstimationStdDevs(estPose, "front");
				RobotContainer.pose.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
			}
		);
		var visionEstBack = RobotContainer.vision.getEstimatedGlobalPose("back");
		visionEstBack.ifPresent(
			est -> {
				var estPose = est.estimatedPose.toPose2d();
				// Change our trust in the measurement based on the tags we can see
				var estStdDevs = RobotContainer.vision.getEstimationStdDevs(estPose, "back");
				RobotContainer.pose.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
			}
		);
	}

    /**
     * addVisionMeasurement fuses the Pose2d from the vision system into the robot pose
     * @param visionMeasurement
     * @param timestampSeconds
     */
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
	}

    /**
     * getAngleOfTarget calculates the vertical angle of the target based on the position of the shooter in the field space
     * @return
     */
	public double getAngleOfTarget() { //TODO: take in enum of targets, move this to its own helper class
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter);
		
		//center of blue speaker, at 6'8.5" up, 9in forward from wall
		Pose3d targetPose = new Pose3d(
			new Translation3d(8.078467, 1.442593, 2.0447),new Rotation3d()
		); 
		
		var shooterToTarget = shooterPose.minus(targetPose);
		var targetAngle = Math.atan(shooterToTarget.getZ()) / (shooterToTarget.getX()); // arctan(height / distance) = radians
		return Math.toDegrees(targetAngle); //change radians to degrees
	}

    /**
     * getHeadingOfTarget calculates the bearing of the target based on the position of the shooter in the field space
     * @return
     */
	public double getHeadingOfTarget() {
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter);
		
		//center of blue speaker, at 6'8.5" up, 9in forward from wall
		Pose3d targetPose = new Pose3d(
			new Translation3d(8.078467, 1.442593, 2.0447),new Rotation3d()
		); 
		
		Transform3d shooterToTarget = shooterPose.minus(targetPose);
		double targetHeading = shooterToTarget.getRotation().getZ(); //radians
		return Math.toDegrees(targetHeading); //change radians to degrees
	}
}
