package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.Robot;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.modules.SwerveModule;

import java.util.ArrayList;
import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * This subsystem handles managing the Drivetrain.
 * It is responsible for controlling the swerve modules based on input from the auton system or the driver.
 */
public class DriveSubsystem extends SubsystemBase {
	private static DriveSubsystem instance;

	//initialize 4 swerve modules
	private static SwerveModule m_frontLeft = new SwerveModule("FL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_frontRight = new SwerveModule("FR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_backLeft = new SwerveModule("RL", Constants.Swerve.BL.constants); // Rear Left
	private static SwerveModule m_backRight = new SwerveModule("RR", Constants.Swerve.BR.constants); // Rear Right
	private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

	private SwerveDrivePoseEstimator poseEstimator;
	private Field2d fieldSim;

	private int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
    SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Pitch"));
	private double sim_last_time = Timer.getFPGATimestamp();

	private double target_heading = 0.0;
	private boolean heading_locked = false;

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		fieldSim = new Field2d();

		//Initialize the pose estimator
		poseEstimator = new SwerveDrivePoseEstimator(
			Constants.Swerve.kDriveKinematics,
			RobotContainer.gyro.getHeading(),
			getSwerveModulePositions(),
			new Pose2d(),
			//stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust your state estimate less.
			VecBuilder.fill(0.1, 0.1, 0.1),
			//visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in radians). Increase these numbers to trust the vision pose measurement less.
			VecBuilder.fill(0.9, 0.9, 0.9)
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
		if(Robot.isSimulation()) updateSim();
		updatePose();
		correctPoseWithVision();
		fieldSim.setRobotPose(poseEstimator.getEstimatedPosition());
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

		builder.addDoubleProperty("Back Left Angle", () -> Helpers.General.roundDouble(m_backLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Left Velocity", () -> Helpers.General.roundDouble(m_backLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Back Right Angle", () -> Helpers.General.roundDouble(m_backRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Right Velocity", () -> Helpers.General.roundDouble(m_backRight.getVelocity(),3), null);

		builder.addDoubleProperty("Robot Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees(), null);
	}

	public void createDashboards() {
		ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
		driverTab.add("Swerve Drive", this)
			.withSize(5, 5)
			.withPosition(19, 4)
			.withProperties(Map.of("show_robot_rotation","true"));
		if(Constants.Swerve.debugDashboard) {
			ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Swerve");
			debugTab.add("Swerve Drive", this)
				.withSize(5, 6)
				.withPosition(0, 0)
				.withProperties(Map.of("show_robot_rotation","true"));
			debugTab.addNumber("FL Angle", () -> Helpers.General.roundDouble(m_frontLeft.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(5, 0);
			debugTab.addNumber("FR Angle", () -> Helpers.General.roundDouble(m_frontRight.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(11, 0);
			debugTab.addNumber("RL Angle", () -> Helpers.General.roundDouble(m_backLeft.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(5, 4);
			debugTab.addNumber("RR Angle", () -> Helpers.General.roundDouble(m_backRight.getAngle().getDegrees(),2))
				.withSize(2, 2)
				.withPosition(11, 4);
			debugTab.addNumber("FL Speed", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(7, 1);
			debugTab.addNumber("FR Speed", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(9, 1);
			debugTab.addNumber("RL Speed", () -> Helpers.General.roundDouble(m_backLeft.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(7, 3);
			debugTab.addNumber("RR Speed", () -> Helpers.General.roundDouble(m_backRight.getVelocity(),3))
				.withSize(2, 2)
				.withPosition(9, 3);
			debugTab.add("Field", fieldSim)
				.withSize(7,4)
				.withPosition(0,6)
				.withWidget("Field")
				.withProperties(Map.of("field_game","Crescendo","robot_width",Units.inchesToMeters(Constants.Global.kBumperWidth),"robot_length",Units.inchesToMeters(Constants.Global.kBumperLength)));
			debugTab.addBoolean("Hdg Locked", this::getHeadingLocked)
				.withSize(2, 2)
				.withPosition(5,2)
				.withWidget("Boolean Box");
			debugTab.addNumber("Hdg Target", this::getTargetHeading)
				.withSize(2, 2)
				.withPosition(7,5);
			debugTab.addNumber("Hdg Err", this::getHeadingError)
				.withSize(2, 2)
				.withPosition(9,5);
		}
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
		};
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	public double getHeadingError() {
		double  error = target_heading - getHeading().getDegrees();
		// error = MathUtil.inputModulus(error, -180.0, 180.0);
		return error;
	}

	public void lockHeading(double heading) {
		target_heading = heading;
		heading_locked = true;
	}

	public void unlockHeading() {
		heading_locked = false;
	}

	public Field2d getField2d() {
		return fieldSim;
	}

	public boolean getHeadingLocked() { return heading_locked; }
	public double getTargetHeading() { return target_heading; }

	/**
	 * Resets the odometry to the specified pose. Requires the current heading to account for starting position other than 0.
	 * 
	 * @param heading The current heading of the robot to offset the zero position
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(double heading, Pose2d pose) {
		RobotContainer.gyro.zeroHeading();
		RobotContainer.gyro.setYawOffset(heading);
		// m_odometry.resetPosition(Rotation2d.fromDegrees(heading), getSwerveModulePositions(), pose);
	}

	/**
     * Reset the estimated pose of the swerve drive on the field.
     *
	 * @param heading Heading to reset robot to (for configuring a yaw offset)
     * @param pose New robot pose.
     */
	public void resetPose(double heading, Pose2d pose) {
		poseEstimator.resetPosition(Rotation2d.fromDegrees(heading), getSwerveModulePositions(), pose);
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
				addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
			}
		);
		var visionEstBack = RobotContainer.vision.getEstimatedGlobalPose("back");
		visionEstBack.ifPresent(
			est -> {
				var estPose = est.estimatedPose.toPose2d();
				// Change our trust in the measurement based on the tags we can see
				var estStdDevs = RobotContainer.vision.getEstimationStdDevs(estPose, "back");
				addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
			}
		);
	}

	public void updateSim() {
		for (SwerveModule module: modules) {
			double ts = Timer.getFPGATimestamp();
			TalonFXSimState driveMotor = module.getDriveMotor().getSimState();
			driveMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
			module.SimDriveMotor.setInputVoltage(addFriction(driveMotor.getMotorVoltage(), 0.25));
			module.SimDriveMotor.update(ts - sim_last_time);
			driveMotor.setRawRotorPosition(module.SimDriveMotor.getAngularPositionRotations() * Constants.Swerve.kRotationsPerWheelRotation);
			driveMotor.setRotorVelocity(module.SimDriveMotor.getAngularVelocityRPM() / 60 * Constants.Swerve.kRotationsPerWheelRotation);
			sim_last_time = ts;
		}
	}

	    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

	/**
	 * Updates the bot pose based on feedback from the drivetrain. This should be done in every periodic loop.
	 */
	public void updatePose() {
		poseEstimator.update(
			RobotContainer.gyro.getHeading(),
			getSwerveModulePositions()
		);
	}

	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}
	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
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
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, RobotContainer.gyro.getHeading()) 
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
		if(!Constants.Swerve.BL.isDisabled) m_backLeft.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.BR.isDisabled) m_backRight.setDesiredState(swerveModuleStates[3]);
	}

	//Stops all modules
	public void brake(boolean withDefensiveLock) {
		if(withDefensiveLock) {
			m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
			m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)));
			m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135.0)));
			m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225.0)));
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
		m_backLeft.setDesiredState(desiredStates[2]);
		m_backRight.setDesiredState(desiredStates[3]);
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
			home &= module.getTurnError() <= Constants.Swerve.kDefaultModuleTurnAllowableError;
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

	public TalonFX[] getMotors() {
	    ArrayList<TalonFX> motors = new ArrayList<>();
		for (SwerveModule module: modules) {
			motors.add(module.getDriveMotor());
		}
		return motors.toArray(new TalonFX[motors.size()]);
	}

	public double getAngleOfTarget() { //TODO: take in enum of targets, move this to its own helper class
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter);
		
		//center of blue speaker, at 6'8.5" up, 9in forward from wall
		Pose3d targetPose = new Pose3d(
			new Translation3d(8.078467, 1.442593, 2.0447),
			new Rotation3d()
		); 
		
		var shooterToTarget = shooterPose.minus(targetPose);
		var targetAngle = Math.atan(shooterToTarget.getZ()) / (shooterToTarget.getX()); // arctan(height / distance) = radians
		return Math.toDegrees(targetAngle); //change radians to degrees
	}

	public double getHeadingOfTarget() {
		Pose3d shooterPose = new Pose3d(poseEstimator.getEstimatedPosition())
			.transformBy(Constants.Shooter.kRobotToShooter);
		
		//center of blue speaker, at 6'8.5" up, 9in forward from wall
		Pose3d targetPose = new Pose3d(
			new Translation3d(8.078467, 1.442593, 2.0447),
			new Rotation3d()
		); 
		
		Transform3d shooterToTarget = shooterPose.minus(targetPose);
		double targetHeading = shooterToTarget.getRotation().getZ(); //radians
		return Math.toDegrees(targetHeading); //change radians to degrees
	}

}