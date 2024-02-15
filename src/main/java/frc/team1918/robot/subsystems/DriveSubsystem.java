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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
		RobotContainer.pose.updatePose();
		RobotContainer.pose.correctPoseWithVision();
		fieldSim.setRobotPose(RobotContainer.pose.getPose());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("SwerveDrive");
		builder.setActuator(true);
		//builder.setSafeState(this::disable); //function for safe state to  make sure things dont move
		builder.addDoubleProperty("Front Left Angle", () -> Helpers.General.roundDouble(-m_frontLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Left Velocity", () -> Helpers.General.roundDouble(m_frontLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Front Right Angle", () -> Helpers.General.roundDouble(-m_frontRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Front Right Velocity", () -> Helpers.General.roundDouble(m_frontRight.getVelocity(),3), null);

		builder.addDoubleProperty("Back Left Angle", () -> Helpers.General.roundDouble(-m_backLeft.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Left Velocity", () -> Helpers.General.roundDouble(m_backLeft.getVelocity(),3), null);

		builder.addDoubleProperty("Back Right Angle", () -> Helpers.General.roundDouble(-m_backRight.getAngle().getDegrees(),2), null);
		builder.addDoubleProperty("Back Right Velocity", () -> Helpers.General.roundDouble(m_backRight.getVelocity(),3), null);

		builder.addDoubleProperty("Robot Angle", () -> RobotContainer.pose.getPose().getRotation().getDegrees(), null);
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
			debugTab.addNumber("Hdg Curr", () -> getHeading().getDegrees())
				.withSize(2, 2)
				.withPosition(11,2);
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

	public Rotation2d getHeading() {
		return RobotContainer.pose.getPose().getRotation();
	}

	public double getHeadingError() {
		double error = target_heading - getHeading().getDegrees();
		// error = MathUtil.inputModulus(error, -180.0, 180.0);
		return error;
	}

	public void lockHeading() {
		//getHeading().getDegrees()
		// target_heading = MathUtil.inputModulus(getHeading().getDegrees(), -180.0, 180.0);
		target_heading = RobotContainer.gyro.getYaw();
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
	public void resetOdometry() { //double heading, Pose2d pose
		RobotContainer.gyro.zeroHeading();
		RobotContainer.gyro.setYawOffset(RobotContainer.isAllianceRed() ? 180 : 0); //set offset to 180 if red
		// RobotContainer.gyro.setYawOffset(heading);
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
	 * Method to drive the robot using percentages of max speeds (from -1.0 to 1.0)
	 * @param xPercent Speed of the robot in the x direction (forward).
	 * @param yPercent Speed of the robot in the y direction (sideways).
	 * @param rotPercent Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	public void drivePercentage(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
		double invert = (RobotContainer.isAllianceRed()) ? -1 : 1; //invert if red
		double xSpeed = invert * xPercent * Constants.DriveTrain.kMaxMetersPerSecond; //positive is away
		double ySpeed = invert * yPercent * Constants.DriveTrain.kMaxMetersPerSecond; //positive is left
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

}