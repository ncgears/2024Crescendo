package frc.team1918.robot.classes;

import java.util.Map;

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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

/**
 * The NCPose class handles getting and managing the different poses and calculations of them.
 * It is responsible for maintaining the robot poses and exposing methods to calculate items related to the poses
 */
public class NCPose {
	private static NCPose instance;
	private SwerveDrivePoseEstimator poseEstimator;

/**
 * April tag positions, in inches
 * ID	X	Y	Z	Rotation
 * 1	593.68	9.68	53.38	120
 * 2	637.21	34.79	53.38	120
 * 3	652.73	196.17	57.13	180
 * 4	652.73	218.42	57.13	180
 * 5	578.77	323.00	53.38	270
 * 6	72.5	323.00	53.38	270
 * 7	-1.50	218.42	57.13	0
 * 8	-1.50	196.17	57.13	0
 * 9	14.02	34.79	53.38	60
 * 10	57.54	9.68	53.38	60
 * 11	468.69	146.19	52.00	300
 * 12	468.69	177.10	52.00	60
 * 13	441.74	161.62	52.00	180
 * 14	209.48	161.62	52.00	0
 * 15	182.73	177.10	52.00	120
 * 16	182.73	146.19	52.00	240
 * April tag positions, in meters
 * ID	X	        Y	        Z	        Rotation    Name
 * 1	15.079472	0.245872	1.355852	120         Blue Source Right
 * 2	16.185134	0.883666	1.355852	120         Blue Source Left
 * 3	16.579342	4.982718	1.451102	180         Red Speaker Right
 * 4	16.579342	5.547868	1.451102	180         Red Speaker Center
 * 5	14.700758	8.2042  	1.355852	270         Red Amp Center
 * 6	1.8415  	8.2042	    1.355852	270         Blue Amp Center
 * 7	-0.0381 	5.547868	1.451102	0           Blue Speaker Center
 * 8	-0.0381 	4.982718	1.451102	0           Blue Speaker Left
 * 9	0.356108	0.883666	1.355852	60          Red Source Right
 * 10	1.461516	0.245872	1.355852	60          Red Source Left
 * 11	11.904726	3.713226	1.3208	    300         Red Trap South (left)
 * 12	11.904726	4.49834	    1.3208	    60          Red Trap North (right)
 * 13	11.220196	4.105148	1.3208	    180         Red Trap Center
 * 14	5.320792	4.105148	1.3208	    0           Blue Trap Center
 * 15	4.641342	4.49834 	1.3208	    120         Blue Trap North (left)
 * 16	4.641342	3.713226	1.3208  	240         Blue Trap South (right)
 */
	/**
	 * Targets represents different locations on the field that we might be interested in tracking
	 */
    public enum Targets { //based on blue origin 0,0 (bottom left) field coordinates, North-East-Up
        SOURCE(0,0,0,120),  //TODO: determine positions
        AMP(1.8415,8.2042,0.889,-90), 
        SPEAKER(0.23,5.547868,2.0447,180),
        STAGE_NORTH(0,0,0,-60),
        STAGE_SOUTH(0,0,0,60),
        STAGE_CENTER(0,0,0,180);
        private final double x,y,z,angle;
        Targets(double x, double y, double z, double angle) { this.x=x; this.y=y; this.z=z; this.angle=angle; }
        public Pose3d getPose() { return new Pose3d(
            new Translation3d(this.x, this.y, this.z),
            new Rotation3d(0,0,Math.toRadians(this.angle))
        ); }
        public Pose3d getMirrorPose() { return new Pose3d(
            new Translation3d(16.54175 - this.x, this.y, this.z), //field is 16.54175 meters
            new Rotation3d(0,0,Math.PI - Math.toRadians(this.angle))
        ); }
    }
	/** State represents different tracking system states */
    public enum State {
        READY("#00FF00"),
        TRACKING("#FFA500"),
        ERROR("#FF0000"),
        STOP("#000000");
        private final String color;
        State(String color) { this.color = color; }
        public String getColor() { return this.color; }
    }
    private State m_trackingState = State.STOP; //current Tracking state
	private Targets m_trackingTarget = Targets.SPEAKER; //current Tracking target

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
		createDashboards();
    }

	public void init() {
		m_trackingState = State.STOP;
		m_trackingTarget = Targets.SPEAKER;
		Helpers.Debug.debug("Pose: Initialized");
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
     * Reset the estimated pose of the swerve drive on the field.
     *
	 * @param heading Heading to reset robot to (for configuring a yaw offset)
     * @param pose New robot pose.
     */
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()), RobotContainer.drive.getSwerveModulePositions(), pose);
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
     * @return the vertical angle of the target, relative to the shooter, in degrees
     */
	public double getAngleOfTarget(Targets target) {
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter);
		// Pose3d targetPose = mirrorPoseIfRed(target.getPose()); //pose of the target
        Pose3d targetPose = (RobotContainer.isAllianceRed()) ? target.getMirrorPose() : target.getPose();
		var shooterToTarget = targetPose.minus(shooterPose);
		// Helpers.Debug.debug("shooter X distance "+shooterToTarget.getX());
		// Helpers.Debug.debug("shooter z radians "+shooterToTarget.getZ());
		var targetAngle = Math.atan(shooterToTarget.getZ()) / (shooterToTarget.getX()); // arctan(height / distance) = radians
		return Math.toDegrees(targetAngle); //change radians to degrees
	}

    /**
     * getBearingOfTarget calculates the bearing of the target based on the position of the shooter in the field space
     * @return the bearing (heading) of the target, relative to the shooter, in degrees
     */
	public double getBearingOfTarget(Targets target) {
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter); //pose of the shooter
		// Pose3d targetPose = mirrorPoseIfRed(target.getPose()); //pose of the target
        Pose3d targetPose = (RobotContainer.isAllianceRed()) ? target.getMirrorPose() : target.getPose();
		Transform3d shooterToTarget = shooterPose.minus(targetPose); //delta from shooter to target
		double targetBearing = shooterToTarget.getRotation().getZ(); //radians
		return Math.toDegrees(targetBearing); //change radians to degrees
	}

	////#region "Tracking"
	/** Creates the dashboard for the tracking system */
	public void createDashboards() {
		if(true) { //false to disable tracking dashboard
			ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Tracking");
			// debugTab.addString("Tracking", this::getTrackingStateColor)
			// 	.withSize(2, 2)
			// 	.withWidget("Single Color View")
			// 	.withPosition(0, 0);  
			// debugTab.addString("State", this::getTrackingStateName)
			// 	.withSize(2,2)
			// 	.withPosition(2,0)
			// 	.withWidget("Text Display");
			// debugTab.addString("Target", this::getTrackingTargetName)
			// 	.withSize(2,2)
			// 	.withPosition(4,0)
			// 	.withWidget("Text Display");
			// debugTab.addNumber("Bearing", this::getTrackingTargetBearing)
			// 	.withSize(2,2)
			// 	.withPosition(0,2);
			// debugTab.addNumber("Angle", this::getTrackingTargetAngle)
			// 	.withSize(2,2)
			// 	.withPosition(2,2);
			ShuffleboardLayout trackingList = debugTab.getLayout("Target Tracking", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(0,0)
				.withProperties(Map.of("Label position","LEFT"));
			trackingList.addString("Tracking", this::getTrackingStateColor)
				.withWidget("Single Color View");
			trackingList.addString("State", this::getTrackingStateName)
				.withWidget("Text Display");
			trackingList.addString("Target", this::getTrackingTargetName)
				.withWidget("Text Display");
			trackingList.addNumber("Bearing", this::getTrackingTargetBearing);
			trackingList.addNumber("Angle", this::getTrackingTargetAngle);
		}
	}
	/** Determines if the robot should be tracking a target
	 * @return boolean indicating if robot is tracking
	 */
	public boolean getTracking() { return m_trackingState != State.STOP; }
	/** Gets the current target tracking state */
	public State getTrackingState() { return m_trackingState; }
	/** Gets the name of the current target tracking state */
	public String getTrackingStateName() { return m_trackingState.toString(); }
	/** Gets the defined color of the current target tracking state */
	public String getTrackingStateColor() { return m_trackingState.getColor(); }
	/** Gets the current tracking target */
	public Targets getTrackingTarget() { return m_trackingTarget; }
	/** Gets the name of the current tracking target */
	public String getTrackingTargetName() { return m_trackingTarget.toString(); }
	/** Gets the relative bearing of the current tracking target */
	public double getTrackingTargetBearing() { return Helpers.General.roundDouble(getBearingOfTarget(m_trackingTarget),2); }
	/** Gets the relative angle from the shooter to the current tracking target */
	public double getTrackingTargetAngle() { return Helpers.General.roundDouble(getAngleOfTarget(m_trackingTarget),2); }
	/** Enables target tracking */
	public void trackingStart() {
		m_trackingState = State.TRACKING;
		Helpers.Debug.debug("Tracking: Start Tracking ("+m_trackingTarget.toString()+")");
    }
	/** Disables target tracking */
    public void trackingStop() {
        m_trackingState = State.STOP;
		Helpers.Debug.debug("Tracking: Stop Tracking");
	}
	/** Sets the requested tracking target
	 * @param target Target to track
	 */
	public void setTrackingTarget(Targets target) { 
		m_trackingTarget = target; 
		Helpers.Debug.debug("Tracking: Set Tracking Target ("+target.toString()+")");
	}
	/** Sets the tracking target to AMP (2024 Crescendo) */
	public void setTrackingAmp() { setTrackingTarget(Targets.AMP); }
	/** Sets the tracking target to SPEAKER (2024 Crescendo) */
	public void setTrackingSpeaker() { setTrackingTarget(Targets.SPEAKER); }
	////#endregion "Tracking"

}
