package frc.team1918.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
// import frc.team1918.robot.Helpers;
import frc.team1918.robot.Robot;
import frc.team1918.robot.modules.SwerveModule;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.controller.PIDController;
//kinematics and odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

@SuppressWarnings("deprecation")
public class DriveSubsystem extends SubsystemBase {
	private static DriveSubsystem instance;
	private static Gyro m_gyro;
	private static Vision m_vision;

	//initialize 4 swerve modules
	private static SwerveModule m_frontLeft = new SwerveModule("FL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_frontRight = new SwerveModule("FR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_rearLeft = new SwerveModule("RL", Constants.Swerve.RL.constants); // Rear Left
	private static SwerveModule m_rearRight = new SwerveModule("RR", Constants.Swerve.RR.constants); // Rear Right
	private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
	private SwerveModulePosition[] swervePositions;

	private SwerveDrivePoseEstimator poseEstimator;
	private Field2d fieldSim;

	//robot theta Controller
	// private PIDController driveStraightPID = new PIDController(Constants.DriveTrain.DriveStraight.kP, Constants.DriveTrain.DriveStraight.kI, Constants.DriveTrain.DriveStraight.kD);

	//intialize odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry;

	// = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getRotation2d());
	//positions(3rd arg) = new SwerveModulePosition(modulePositions[index].distanceMeters, modulePositions[index].angle);
	//initial pose (4th arg) = Pose2d of position on field

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		fieldSim = new Field2d();
		//Perform init function on each module, not needed?
		// for (SwerveModule module: modules) {
		// 	module.resetDistance();
		// 	module.syncTurningEncoders();
		// }

		//Get the positions of the swerve modules
		swervePositions = getSwerveModulePositions();

		//Initialize the gyro object
		m_gyro = new Gyro();

		//Initialize the odometry object
		m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getHeading(), swervePositions);

		//Initialize the vision object
		m_vision = new Vision();

		//Initialize the pose estimator
		poseEstimator = new SwerveDrivePoseEstimator(
			Constants.Swerve.kDriveKinematics,
			m_gyro.getHeading(),
			getSwerveModulePositions(),
			getPose(),
			//stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust your state estimate less.
			VecBuilder.fill(.05, .05, Units.degreesToRadians(5)), //.1,.1,.1
			//visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
			VecBuilder.fill(.5, .5, Units.degreesToRadians(30)) //.9,.9,.9
		);
		
		// m_targetPose = m_odometry.getPoseMeters();
		// m_thetaController.reset();
		// m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

		//Add this sendable to the Dashboard
		//SmartDashboard.putData("Swerve Drive", this);
		createDashboards();
	}

	// @SuppressWarnings("unused")
	@Override
	public void periodic() {
		updateOdometry();
		updateDashboard();
		if(Robot.isSimulation()) {
            //This is for updating simulated data
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("SwerveDrive");
		builder.setActuator(true);
		//builder.setSafeState(this::disable); //function for safe state to  make sure things dont move
		builder.addDoubleProperty("Front Left Angle", () -> Helpers.General.roundDouble(m_frontLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Left Velocity", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Front Right Angle", () -> Helpers.General.roundDouble(m_frontRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Right Velocity", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3), null);

		builder.addDoubleProperty("Back Left Angle", () -> Helpers.General.roundDouble(m_rearLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Left Velocity", () -> Helpers.General.roundDouble(m_rearLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Back Right Angle", () -> Helpers.General.roundDouble(m_rearRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Right Velocity", () -> Helpers.General.roundDouble(m_rearRight.getVelocity(),3), null);

		builder.addDoubleProperty("Robot Angle", () -> m_gyro.getHeading().getDegrees(), null);
	}

	public void createDashboards() {
		ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
		driverTab.add("Swerve Drive", this)
			.withSize(5, 5)
			.withPosition(19, 4);
		if(Constants.Swerve.debugDashboard) {
			ShuffleboardTab debugTab = Shuffleboard.getTab("Debug: Swerve");
			debugTab.add("Swerve Drive", this)
				.withSize(5, 6)
				.withPosition(0, 0);
			debugTab.addNumber("FL Angle", () -> Helpers.General.roundDouble(m_frontLeft.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(5, 0);
			debugTab.addNumber("FR Angle", () -> Helpers.General.roundDouble(m_frontRight.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(11, 0);
			debugTab.addNumber("RL Angle", () -> Helpers.General.roundDouble(m_rearLeft.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(5, 4);
			debugTab.addNumber("RR Angle", () -> Helpers.General.roundDouble(m_rearRight.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(11, 4);
			debugTab.addNumber("FL Speed", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(7, 1);
			debugTab.addNumber("FR Speed", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(9, 1);
			debugTab.addNumber("RL Speed", () -> Helpers.General.roundDouble(m_rearLeft.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(7, 3);
			debugTab.addNumber("RR Speed", () -> Helpers.General.roundDouble(m_rearRight.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(9, 3);

		}
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
		};
	}

	public void updateDashboard() {
		// Dashboard.DriveTrain.setHeading(m_gyro.getHeading().getDegrees());
		// Dashboard.DriveTrain.setX(getPose().getX());
		// Dashboard.DriveTrain.setY(getPose().getY());
		// Dashboard.DriveTrain.setCurrentAngle(getPose().getRotation().getDegrees());
		// Dashboard.DriveTrain.setDesiredAngle(desiredAngle);
		// Dashboard.DriveTrain.setCommunity(inCommunity);
		// Dashboard.DriveTrain.setTargetAngle(m_targetPose.getRotation().getRadians());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Field2d getField2d() {
		return fieldSim;
	}

	public void resetEstimatedPose(Pose2d pose) {
		poseEstimator.resetPosition(m_gyro.getHeading(), swervePositions, pose);
		fieldSim.setRobotPose(getEstimatedPose());
	}

	public void addVisionPose() {
		Pose2d visionPose = m_vision.getPose();
		double visionTimestamp = m_vision.getTimestamp();
		if ((m_vision.getNumTags() == 1 && m_vision.getDistance() < 75.00) ||
			(m_vision.getNumTags() == 2 && m_vision.getDistance() < 120.00) ||
			(m_vision.getNumTags() == 3 && m_vision.getDistance() < 185.00)
		) {
			poseEstimator.addVisionMeasurement(visionPose, visionTimestamp);
		}
	}

	/**
	 * Resets the odometry to the specified pose. Requires the current heading to account for starting position other than 0.
	 * 
	 * @param heading The current heading of the robot to offset the zero position
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(double heading, Pose2d pose) {
	  m_gyro.zeroHeading();
	  m_gyro.setYawOffset(heading);
	  m_odometry.resetPosition(Rotation2d.fromDegrees(heading), getSwerveModulePositions(), pose);
	}

	/**
	 * Updates the odometry of the robot. This should be done in every periodic loop.
	 */
	public void updateOdometry() {
		m_odometry.update(
			m_gyro.getHeading(),
			new SwerveModulePosition[] {
        	    m_frontLeft.getPosition(),
            	m_frontRight.getPosition(),
            	m_rearLeft.getPosition(),
            	m_rearRight.getPosition()
			}
		);
		fieldSim.setRobotPose(m_odometry.getPoseMeters());
		//TODO: See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervedriveposeestimator/Drivetrain.java
		//TODO: Using a pose estimator will allow us to fuse multiple data into the odometry
		// if(Constants.Vision.useForOdometry) {
		// 	m_odometry.addVisionMeasurement(
		// 		m_vision.getEstimatedGlobalPose(m_odometry.getEstimatedPosition()),
		// 		Timer.getFPGATimestamp() - 0.3
		// 	);
		// }
	}



	/**
	 * Method to drive the robot using percentages of max speeds (from -1.0 to 1.0)
	 * @param xPercent Speed of the robot in the x direction (forward).
	 * @param yPercent Speed of the robot in the y direction (sideways).
	 * @param rotPercent Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drivePercentage(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
		double xSpeed = xPercent * Constants.DriveTrain.kMaxMetersPerSecond;
		double ySpeed = yPercent * Constants.DriveTrain.kMaxMetersPerSecond;
		double rot = rotPercent * Constants.DriveTrain.kMaxRotationRadiansPerSecond;
		drive(xSpeed, ySpeed, rot, fieldRelative);
	}

	/**
	 * Method to drive the robot using calculated speed info.
	 * @param xSpeed Speed of the robot in the x direction (forward).
	 * @param ySpeed Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	// @SuppressWarnings("unused")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		ChassisSpeeds speeds = ChassisSpeeds.discretize(
			(fieldRelative) 
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getHeading()) 
				: new ChassisSpeeds(xSpeed, ySpeed, rot),
			Robot.kDefaultPeriod);
			drive(speeds,true);
	}
	/**
	 * Method to drive the robot using calculated speed info.
	 * @param speeds ChassisSpeeds object representing the x,y and rotational speed of the robot
	 * @param normalize Boolean value indicating whether the speeds should be normalized such that none of them are over 100%
	 */
	@SuppressWarnings("unused")
	public void drive(ChassisSpeeds speeds, boolean normalize) {
		if (Constants.DriveTrain.useBrakeWhenStopped && (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0)) {
			brake(false);
		}
		SwerveModuleState[] swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(speeds);
		if (normalize) SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// setModuleStates(swerveModuleStates);
		if(!Constants.Swerve.FL.isDisabled) m_frontLeft.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.FR.isDisabled) m_frontRight.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.RL.isDisabled) m_rearLeft.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.RR.isDisabled) m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	//Stops all modules
	public void brake(boolean withDefensiveLock) {
		if(withDefensiveLock) {
			m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
			m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)));
			m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135.0)));
			m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225.0)));
		} else {
			for (SwerveModule module: modules) {
				module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
			}
		}
	}

	/**
	 * Sets the swerve ModuleStates.
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to read a position of 0. */
	public void resetEncoders() {
		for (SwerveModule module: modules) {
			module.resetEncoders();
		}
	}

	/** Resets the drive distances to read 0. */
	public void resetDistances() {
		for (SwerveModule module: modules) {
			module.resetDistance();
		}
	}

	/** Moves the swerve modules to their 0 position (in current loop). */
	public void homeSwerves() {
		for (SwerveModule module: modules) {
			module.homeSwerve();
		}
	}

	public boolean swervesAtHome() {
		boolean home = true;
		for (SwerveModule module: modules) {
			home &= module.getTurnError() <= Constants.Swerve.DEFAULT_TURN_ALLOWED_ERROR;
		}
		return home;
	}

	//#region MOTOR CONTROLLER STUFF
	public void setAllDriveBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("drive",b);
		}
	}

	public void setAllTurnBrakeMode(boolean b) {
		for (SwerveModule module: modules) {
			module.setBrakeMode("turn", b);
		}
	}
	//#endregion MOTOR CONTROLLER STUFF
}