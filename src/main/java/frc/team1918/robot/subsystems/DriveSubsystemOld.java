package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.Robot;
import frc.team1918.robot.modules.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
//kinematics and odometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.geometry.Translation2d;

@Deprecated(since="2024")
public class DriveSubsystemOld extends SubsystemBase {

	private static DriveSubsystemOld instance;
	//private static double l = Constants.Global.ROBOT_LENGTH, w = Constants.Global.ROBOT_WIDTH, r = Math.sqrt((l * l) + (w * w));
	private int debug_ticks;
	private static double desiredAngle; //Used for driveStraight function
	private static boolean angleLocked = false;
	//private double[] lastDistances;
	private SwerveModulePosition[] swervePositions;
	private Pose2d robotPose;
	//private double lastTime;
	//private final Timer timer;
	private static double yawOffset = 0.0; //offset to account for different starting positions
	public boolean visionTargeting = false;
	private PIDController driveStraightPID = new PIDController(Constants.DriveTrain.DriveStraight.kP, Constants.DriveTrain.DriveStraight.kI, Constants.DriveTrain.DriveStraight.kD);

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule("dtFL", Constants.Swerve.FL.constants); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule("dtFR", Constants.Swerve.FR.constants); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule("dtRL", Constants.Swerve.RL.constants); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule("dtRR", Constants.Swerve.RR.constants); // Rear Right
	private SwerveModule[] modules = {m_dtFL, m_dtFR, m_dtRL, m_dtRR};

	//initialize gyro object
	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private final Field2d m_2dField;
	//intialize odometry class for tracking robot pose
	static SwerveDriveOdometry m_odometry; // = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getRotation2d());
	//positions(3rd arg) = new SwerveModulePosition(modulePositions[index].distanceMeters, modulePositions[index].angle);
	//initial pose (4th arg) = Pose2d of position on field

	public static DriveSubsystemOld getInstance() {
		if (instance == null)
			instance = new DriveSubsystemOld();
		return instance;
	}

	public DriveSubsystemOld() { //initialize the class
		//Deprecated in 2024? // m_gyro.calibrate();
		driveStraightPID.enableContinuousInput(0,360);
		m_2dField = new Field2d();
		for (SwerveModule module: modules) {
			module.resetDistance();
			module.syncTurningEncoders();
		}

		//Get the positions of the swerve modules
		swervePositions = getSwerveModulePositions();

		//Initialize the odometry object
		m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, getHeading(), swervePositions);

		// m_targetPose = m_odometry.getPoseMeters();
		// m_thetaController.reset();
		// m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
/* not needed in 2023
		lastDistances = new double[]{
			m_dtFL.getDriveDistanceMeters(),
			m_dtFR.getDriveDistanceMeters(),
			m_dtRL.getDriveDistanceMeters(),
			m_dtRR.getDriveDistanceMeters()
		};
		timer = new Timer();
		timer.reset();
		timer.start();
		lastTime = 0;
*/
	}

	@SuppressWarnings("unused")
	@Override
	public void periodic() {
		updateOdometry();
		updateDashboard();
		for (SwerveModule module: modules) {
			// module.updateDashboard();
		}
		if(Robot.isSimulation()) {
			//not working, but if simulated, lets pretend to go somewhere
			//drive(0.2,0,0.0,true); //drive somewhere for simulation
		}
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		return new SwerveModulePosition[] {
            m_dtFL.getPosition(),
            m_dtFR.getPosition(),
            m_dtRL.getPosition(),
            m_dtRR.getPosition()
		};
	}

	public void updateDashboard() {
		Dashboard.DriveTrain.setHeading(getHeading().getDegrees());
		Dashboard.DriveTrain.setX(getPose().getX());
		Dashboard.DriveTrain.setY(getPose().getY());
		Dashboard.DriveTrain.setCurrentAngle(getPose().getRotation().getDegrees());
		Dashboard.DriveTrain.setDesiredAngle(desiredAngle);
		Dashboard.Gyro.setGyroPitch(getGyroPitch());
		// Dashboard.DriveTrain.setTargetAngle(m_targetPose.getRotation().getRadians());
	}

	public void setVisionTargeting(boolean enabled) {
		visionTargeting = enabled;
	}

	/**
     * Returns the heading of the robot.
     * @return the robot's heading as a Rotation2d
     */
	public Rotation2d getHeading() {
		double raw_yaw = m_gyro.getYaw() - (double)yawOffset;  //always subtract the offset
		double calc_yaw = raw_yaw;
		if (0.0 > raw_yaw) { //yaw is negative
			calc_yaw += 360.0;
		}
		calc_yaw *= (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
		return Rotation2d.fromDegrees(calc_yaw);
	}

	public void zeroHeading() {
		m_gyro.zeroYaw();
		yawOffset = 0;
	}

		/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose. Requires the current heading to account for starting position other than 0.
	 * 
	 * @param heading The current heading of the robot to offset the zero position
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(double heading, Pose2d pose) {
	  zeroHeading();
	  yawOffset = heading;
	  m_odometry.resetPosition(Rotation2d.fromDegrees(heading), getSwerveModulePositions(), pose);
	}

	public void updateOdometry() {
		/*This is the 2022 way
		double[] distances = new double[] {
			m_dtFL.getDriveDistanceMeters(),
			m_dtFR.getDriveDistanceMeters(),
			m_dtRL.getDriveDistanceMeters(),
			m_dtRR.getDriveDistanceMeters()
		};
		double time = timer.get();
		double dt = time - lastTime;
		lastTime = time;
		if (dt == 0) return;
		m_odometry.updateWithTime(time,
			getHeading(),
			new SwerveModuleState((distances[0] - lastDistances[0]) / dt, m_dtFL.getState().angle),
			new SwerveModuleState((distances[1] - lastDistances[1]) / dt, m_dtFR.getState().angle),
			new SwerveModuleState((distances[2] - lastDistances[2]) / dt, m_dtRL.getState().angle),
			new SwerveModuleState((distances[3] - lastDistances[3]) / dt, m_dtRR.getState().angle)
		);
		lastDistances = distances;
		*/
		/*This is the 2023 way */
		robotPose = m_odometry.update(
			getHeading(),
			new SwerveModulePosition[] {
        	    m_dtFL.getPosition(),
            	m_dtFR.getPosition(),
            	m_dtRL.getPosition(),
            	m_dtRR.getPosition()
			}
		);
		m_2dField.setRobotPose(robotPose); //This updates the Field2d with odometry of robot
	}

	public Pose2d getPose2d() {
		return robotPose;
	}

	public void lockAngle() {
		desiredAngle = getHeading().getDegrees(); //Helpers.General.roundDouble(m_gyro.getAngle(), 3);
		angleLocked = true;
		Helpers.Debug.debug("Angle Locked to "+Helpers.General.roundDouble(desiredAngle,3));
	}

	public void unlockAngle() {
		if(angleLocked) Helpers.Debug.debug("Angle Unlocked");
		angleLocked = false;
	}

	/**
	 * Method to drive the robot using joystick info.
	 * @param fwd Speed of the robot in the x direction (forward).
	 * @param str Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double fwd, double str, double rot, boolean fieldRelative) {
		if(!Constants.DriveTrain.DriveStraight.isDisabled) {
			if(rot == 0) { //We are not applying rotation
				if(!angleLocked) { //We havent locked an angle, so lets save the desiredAngle and lock it
					lockAngle();
				} else {
					if (Math.abs(fwd) > 0 || Math.abs(str) > 0) { //Only do angle correction while moving, for safety reasons
						double pidVal = driveStraightPID.calculate(getHeading().getDegrees(),desiredAngle);
						Dashboard.DriveTrain.setCorrectionAngle(pidVal);
						rot += pidVal;
						// Dashboard.DriveTrain.setCorrectionAngle(calcAngleStraight(desiredAngle,getHeading().getDegrees(),Constants.DriveTrain.kDriveStraight_P));
						// rot += calcAngleStraight(desiredAngle,getHeading().getDegrees(),Constants.DriveTrain.kDriveStraight_P); //Add some correction to the rotation to account for angle drive
					}
				}
			}
		}
		double fwdMPS = fwd * Constants.DriveTrain.kMaxMetersPerSecond;
		double strMPS = str * Constants.DriveTrain.kMaxMetersPerSecond;
		double rotRPS = rot * Constants.DriveTrain.kMaxRotationRadiansPerSecond;
		//getHeading was getRot2d which used m_gyro.getAngle()
		ChassisSpeeds speeds = (fieldRelative) ? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rotRPS, getHeading()) : new ChassisSpeeds(fwdMPS, strMPS, rotRPS);
		if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
			brake(false);
			return;
		}
		if (Helpers.Debug.debugThrottleMet(debug_ticks)) {
			Helpers.Debug.debug( (fieldRelative) ? speeds.toString()+" fieldCentric" : speeds.toString()+" robotCentric");
		}
		debug_ticks++;
		var swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		if(!Constants.Swerve.FL.isDisabled) m_dtFL.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.FR.isDisabled) m_dtFR.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.RL.isDisabled) m_dtRL.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.RR.isDisabled) m_dtRR.setDesiredState(swerveModuleStates[3]);
	}
	public void drive(ChassisSpeeds speeds, boolean normalize) {
		if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
			brake(false);
			return;
		}
		SwerveModuleState[] swerveModuleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(speeds);
		if (normalize) SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		// setModuleStates(swerveModuleStates);
		if(!Constants.Swerve.FL.isDisabled) m_dtFL.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.FR.isDisabled) m_dtFR.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.RL.isDisabled) m_dtRL.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.RR.isDisabled) m_dtRR.setDesiredState(swerveModuleStates[3]);
	}

	//Stops all modules
	public void brake(boolean withDefensiveLock) {
		if(withDefensiveLock) {
			m_dtFL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45.0)));
			m_dtFR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315.0)));
			m_dtRL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135.0)));
			m_dtRR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225.0)));
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
		m_dtFL.setDesiredState(desiredStates[0]);
		m_dtFR.setDesiredState(desiredStates[1]);
		m_dtRL.setDesiredState(desiredStates[2]);
		m_dtRR.setDesiredState(desiredStates[3]);
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
		// boolean[] atHome;
		// int i = 0;
		for (SwerveModule module: modules) {
			// atHome[i] = module.getTurnError() <= Constants.Swerve.DEFAULT_TURN_ALLOWED_ERROR;
			home &= module.getTurnError() <= Constants.Swerve.DEFAULT_TURN_ALLOWED_ERROR;
			// i++;
		}
		// for (int j=0; j<atHome.length; j++) {
			
		// }
		return home;
	}

	/**
	 * This returns and angle correction (in degrees)
	 * @param targetAngle [double] target angle (heading) of the robot in degrees
	 * @param currentAngle [double] current angle (heading) of the robot in degrees
	 * @param kP [double] proportional multiplier for straight angle correction
	 * @return [double] Degrees of correction using kP multiplier (to control how quickly we correct back to straight)
	 */
	// TODO: Build actual pid controller for this
	public double calcAngleStraight(double targetAngle, double currentAngle, double kP) {
		double errorAngle = (targetAngle - currentAngle) % 360.0;
		double correction = errorAngle * kP;
		// if (Math.abs(correction) > .5) return Math.signum(correction) * 0.01;
		return correction;
	}

	public Field2d getField() {
		return m_2dField;
	}

	//#region GYRO STUFF
	public AHRS getGyro() {
        return m_gyro;
	}

	public double getGyroPitch() {
		return m_gyro.getPitch();
	}

	public double getGyroYaw() {
		return m_gyro.getYaw();
	}

	public double getGyroRoll() {
		return m_gyro.getRoll();
	}
	//#endregion GYRO STUFF

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