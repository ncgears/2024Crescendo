package frc.team1918.robot.modules;

//Talon SRX/Talon FX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

//WPILib
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModule {
    private WPI_TalonSRX turn;
    public TalonFX drive;
    public final DCMotorSim SimDriveMotor, SimSteerMotor;
    //private final double FULL_ROTATION = Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;
    private final double TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    private final int TURN_ALLOWED_ERROR;
    private String moduleName;
    private double driveWheelDiamMM = Constants.Swerve.kDefaultModuleWheelDiamMM;
    private NeutralOut m_brake = new NeutralOut();
    private SwerveModuleState state;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        Constants.Swerve.kDriveKS, 
        Constants.Swerve.kDriveKV, 
        Constants.Swerve.kDriveKA);

 	/**
	 * 1918 Swerve Module v2024.1 - This swerve module uses a Falcon 500 (TalonFX) for drive and Talon SRX for turn (bag motor with gearbox).
     * The module uses a Lamprey Absolute encoder for positioning data
	 * @param name This is the name of this swerve module (ie. "dtFL")
     * @param moduleConstants This is a SwerveModuleConstants object containing the data for this module
	 */
    public SwerveModule(String name, SwerveModuleConstants moduleConstants){
        moduleName = name;
        drive = new TalonFX(moduleConstants.idDriveMotor, Constants.Swerve.canBus);
        turn = new WPI_TalonSRX(moduleConstants.idTurnMotor);
        TURN_P = moduleConstants.turnP;
        TURN_I = moduleConstants.turnI;
        TURN_D = moduleConstants.turnD;
        TURN_IZONE = moduleConstants.turnIZone;
        TURN_ALLOWED_ERROR = moduleConstants.turnMaxAllowedError;
        driveWheelDiamMM = moduleConstants.driveWheelDiamMM;
        SimDriveMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.kRotationsPerWheelRotation, moduleConstants.DriveInertia);
        SimSteerMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.Swerve.kRotationsPerWheelRotation, moduleConstants.DriveInertia);

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.Analog, //  FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
                                            Constants.Global.kPidIndex,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
        turn.configFeedbackNotContinuous(Constants.Global.SWERVE_SENSOR_NONCONTINUOUS, 0); //Disable continuous feedback tracking (so 0 and 1024 are effectively one and the same)

        // turn.setSelectedSensorPosition(0); //reset the talon encoder counter to 0 so we dont carry over a large error from a previous testing
        // turn.set(ControlMode.Position, 1024); //set this to some fixed value for testing
        turn.setSensorPhase(moduleConstants.turnSensorPhase); //set the sensor phase based on the constants setting for this module
        turn.setInverted(moduleConstants.turnIsInverted); //set the motor direction based on the constants setting for this module
        turn.config_kP(Constants.Global.kPidProfileSlotIndex, TURN_P); //set the kP for PID Tuning
        turn.config_kI(Constants.Global.kPidProfileSlotIndex, TURN_I);
        turn.config_kD(Constants.Global.kPidProfileSlotIndex, TURN_D);
        turn.config_IntegralZone(Constants.Global.kPidProfileSlotIndex, TURN_IZONE);
        turn.overrideLimitSwitchesEnable(false);
        turn.configAllowableClosedloopError(Constants.Global.kPidProfileSlotIndex, TURN_ALLOWED_ERROR); 
        if(Constants.Swerve.homeOnInit) turn.set(ControlMode.Position, 0);
        // SupplyCurrentLimitConfiguration(enabled,peak,trigger threshold current,trigger threshold time(s))
        // turn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.isTurnCurrentLimitEnabled,
        //     Constants.Swerve.kTurnCurrentLimitAmps,
        //     Constants.Swerve.kTurnCurrentThresholdAmps,
        //     Constants.Swerve.kTurnCurrentThresholdSecs));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = drive.getConfigurator().apply(RobotContainer.ctreConfigs.swerveDriveFXConfig);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            Helpers.Debug.debug("Could not initialize swerve module " + name + ", error: " + status.toString());
        }
        drive.setInverted(moduleConstants.driveIsInverted);
        // drive.getVelocity().setUpdateFrequency(50); //example of changing update signal frequency, 0 disables the frame
        // drive.optimizeBusUtilization(0.2); //clean up the bus of all not explicitly requested signals
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        if (state != null) return state;
        return new SwerveModuleState(0, getTurnPositionAsRotation2d());
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins.  If the desired destination angle is more than 90 degrees either direction from the current position, 
     * instead choose a new destination that is 180 degrees from the desired destination, but invert the drive motor speed.
     * @param desiredState The desired state.
     * @return Swerve Module State reflecting the shortest path to the desired turn and the correct drive speed
     */
    public SwerveModuleState optimize(SwerveModuleState desiredState) { 
        Rotation2d currentAngle = getTurnPositionAsRotation2d();
        double delta = deltaAdjustedAngle(desiredState.angle.getDegrees(), currentAngle.getDegrees());
        double driveOutput = desiredState.speedMetersPerSecond;
        if (Math.abs(delta) > 90) { //if the requested delta is greater than 90 degrees, invert drive speed and use 180 degrees from desired angle
            driveOutput *= -1;
            delta -= Math.signum(delta) * 180;
        }
        Rotation2d adjustedAngle = Rotation2d.fromDegrees(delta + currentAngle.getDegrees());
        return new SwerveModuleState(driveOutput, adjustedAngle);
    }

    /**
     * This function takes a desiredState and instructs the motor controllers to move based on the desired state
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        state = (Constants.Swerve.useTurnOptimization) ? optimize(desiredState) : desiredState;

        double percentOutput = state.speedMetersPerSecond / Constants.Swerve.kMaxSpeedMetersPerSecond; //Create a percentage from the theoretical max
        setDrivePower(percentOutput);
        // setDrivePower(feedforward.calculate(state.speedMetersPerSecond));

        int turn_ticks = Helpers.General.radiansToTicks(state.angle.getRadians() + Constants.Swerve.kHomeOffsetRadians);
        turn.set(ControlMode.Position, turn_ticks);
    }

    /**
     * This function sets the turn power by percentage
	 * @param power turn power from -1.0 to 1.0
    */
    public void setTurnPower(double power){
        turn.set(ControlMode.PercentOutput, power);
    }

    /**
     * This function sets the drive power by percentage
	 * @param power drive motor power from -1.0 to 1.0
    */
    public void setDrivePower(double power){
        drive.setControl(new DutyCycleOut(power, true, false, false, false));
    }

    public void homeSwerve() {
        turn.set(ControlMode.Position, 0);
    }

    public void defensiveLock(double position) {
        turn.set(ControlMode.Position, position);
    }

    /**
     * Returns a SwerveModulePosition object representing the distance and angle of the module
     * @return SwerveModulePosition object of the distance(meters) and angle(rotation2d)
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getTurnPositionAsRotation2d().unaryMinus());
    }

    /**
     * Returns a rotation2d object representing the current location of the turn sensor
     * @return Rotation2d object of the current position
     */
    public Rotation2d getTurnPositionAsRotation2d(){
        int pos = (int) turn.getSelectedSensorPosition(0); //not normalized
        // int pos = getTurnPosition();
        return new Rotation2d(Helpers.General.ticksToRadians(pos));
    }

    public Rotation2d getAngle() {
        // return getState().angle.unaryMinus();
        var angle = getState().angle.unaryMinus().getDegrees();
        angle = ((angle + 180) % 360 + 360) % 360 - 180;
        return Rotation2d.fromDegrees(angle);
    }
    public double getVelocity() {
        return getState().speedMetersPerSecond;
    }

    /**
     * Gets the closed-loop error. The units depend on which control mode is in use. If closed-loop is seeking a target sensor position, closed-loop error
     * is the difference between target and current sensor value (in sensor units. Example 4096 units per rotation for CTRE Mag Encoder). 
     * If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target and current sensor value 
     * (in sensor units per 100ms). If using motion profiling or Motion Magic, closed loop error is calculated against the current target, 
     * and not the "final" target at the end of the profile/movement. See Phoenix-Documentation information on units.
     * @return Double precision units of error
     */
    public double getTurnError() {
        return turn.getClosedLoopError(0);
    }

    /**
     * Stops both turning and driving by setting their respective motor power to 0.
     */
    public void stopBoth() {
        setDrivePower(0);
        setTurnPower(0);
    }

    /**
     * Sets the brake mode for the motor controller
     * @param device String of either "turn" or "drive" indicating which device to set
     * @param brake Boolean indicating if the brake mode should be set to brake (true) or coast (false)
     */
    public void setBrakeMode(String device, boolean brake) {
        switch (device) {
            case "turn": //turn is a TalonSRX
                if (brake) {
                    turn.setNeutralMode(NeutralMode.Brake);
                } else {
                    turn.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case "drive": //drive is a TalonFX
                if (brake) {
                    drive.setControl(m_brake);
                }
                break;
        }
    }

    public String getModuleName() {
        return moduleName;
    }

    /**
     * calculate the turning motor setpoint based on the desired angle and the current angle measurement
     * @param targetAngle desired target in radians
     * @param currentAngle current angle in radians
     * @return Delta angle in radians
     */
    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public double getDriveDistanceMeters() {
        // return Helpers.General.encoderToMeters(drive.getPosition().getValue(), this.driveWheelDiamMM);
        double wheel_rotations = drive.getPosition().getValue(); //Uses configured SensorToMechanismRatio in CTREConfigs
        double distance = wheel_rotations * Math.PI * driveWheelDiamMM / 1000;
        return distance;
    }

    public void resetDistance() {
        drive.setPosition(0.0);
    }

    public void resetEncoders() {
        // this resets cumulative rotation counts to zero, resets the position of the turn encoders
        // primarily used with an external encoder such as cancoder, does nothing with our lampreys
    }

    public void syncTurningEncoders() {
        // turn.setSelectedSensorPosition(turn.getSelectedSensorPosition(0),1,0);
    }

    public TalonFX getDriveMotor() {
        return drive;
    }

    public WPI_TalonSRX getSteerMotor() {
        return turn;
    }
}