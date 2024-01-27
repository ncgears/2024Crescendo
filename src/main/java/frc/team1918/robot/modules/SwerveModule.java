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
import frc.team1918.robot.Constants;
import frc.team1918.robot.DashboardOld;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

@Deprecated(since="2024")
public class SwerveModule {
    private WPI_TalonSRX turn;
    private TalonFX drive;
    //private final double FULL_ROTATION = Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;
    private final double TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    private final int TURN_ALLOWED_ERROR;
    private String moduleName;
    private double driveWheelDiam = Constants.Swerve.DEFAULT_WHEEL_DIAM_MM;
    private NeutralOut m_brake = new NeutralOut();
    private SwerveModuleState state;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

 	/**
	 * 1918 Swerve Module v2022.1 - This swerve module uses a Falcon 500 (TalonFX) for drive and Talon SRX for turn (bag motor with gearbox).
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
        driveWheelDiam = moduleConstants.driveWheelDiam;

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
        if(Constants.Swerve.homeOnInit) turn.set(ControlMode.Position, getZeroPositionTicks());
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


        // drive.configFactoryDefault(); //v5
        // // drive.getConfigurator().apply(new TalonFXConfiguration()); //v6
        // drive.set(ControlMode.PercentOutput, 0);
        // drive.setNeutralMode(NeutralMode.Brake);
        // drive.setInverted(moduleConstants.driveIsInverted);
        // drive.config_kP(Constants.Global.kPidProfileSlotIndex, 0.0005);
        // drive.config_kI(Constants.Global.kPidProfileSlotIndex, 0.0);
        // drive.config_kD(Constants.Global.kPidProfileSlotIndex, 0.00005);
        // drive.config_IntegralZone(Constants.Global.kPidProfileSlotIndex, 4740);
        // SupplyCurrentLimitConfiguration(enabled,peak,trigger threshold current,trigger threshold time(s))
        // drive.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,60.0,45.0,1.0));
        // drive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.isDriveCurrentLimitEnabled,
        //     Constants.Swerve.kDriveCurrentLimitAmps,
        //     Constants.Swerve.kDriveCurrentThresholdAmps,
        //     Constants.Swerve.kDriveCurrentThresholdSecs));
            
        // m_drive_pidController = drive.getPIDController();
        // m_drive_pidController.setP(0.0005); //PID P
        // m_drive_pidController.setI(0.0); //PID I
        // m_drive_pidController.setD(0.00005); //PID D
        // // m_drive_pidController.setIZone(0); //IZone
        // m_drive_pidController.setFF(1/4740); //Feed forward
        // m_drive_pidController.setOutputRange(-1, 1);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // double rps = drive.getVelocity().getValueAsDouble();
        // double wheelRps = Helpers.General.gearCalcDouble(rps,
        //     Constants.DriveTrain.DT_DRIVE_FIRST_GEARONE,
        //     Constants.DriveTrain.DT_DRIVE_FIRST_GEARTWO,
        //     Constants.DriveTrain.DT_DRIVE_SECOND_GEARONE,
        //     Constants.DriveTrain.DT_DRIVE_SECOND_GEARTWO
        // );
        // return new SwerveModuleState(Helpers.General.rpsToMetersPerSecond(wheelRps, this.driveWheelDiam), getTurnPositionAsRotation2d());
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
        // drive.set(percentOutput);
        setDrivePower(percentOutput);

        int turn_ticks = Helpers.General.radiansToTicks(state.angle.getRadians() + Constants.Swerve.kHomeOffsetRadians);
        turn.set(ControlMode.Position, turn_ticks);

        // //Display output for debugging
        // if(Helpers.Debug.debugThrottleMet(debug_ticks1) && state.speedMetersPerSecond != 0.0) {
        //     Helpers.Debug.debug(moduleName+" Speed (metersPerSecond)="+Helpers.General.roundDouble(state.speedMetersPerSecond,3)+" Turn Setpoint="+turn_ticks);
        // }
        // debug_ticks1++;
    }

    /**
     * This function calculates the minimum turn (in encoder ticks) based on the desired location and the current location.
     * It uses the encoder wrap-around to determine if we should turn negative past 0
     * @param desiredPosition encoder count of desired position
     * @param currentPosition encoder count of current position
     * @return An encoder count between -1024..0..1024 of the new target
     */
    public int minTurnTicks(int desiredPosition, int currentPosition) {
        int wrap = (int) Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION; //This is where the encoder changes from high to 0
        return Helpers.General.minChange(desiredPosition,currentPosition,wrap);
        //minChange calculates the shortest distance to the desired target, even if that means wrapping over 0.
        //Examples based on 4096 (SRX Mag Encoder)
        //Example: curTicks = 1023 (45deg), desiredTicks = 3073 (1 tick above 270deg)
        //Example result: -2047 ticks (-180deg) instead of +2049 ticks
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

    public int getZeroPositionTicks() {
        int total = (int) turn.getSelectedSensorPosition(0);
        int curr = (int) total & Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;//0x3ff; //ticks in current rotation
        return total - curr;
    }

    public void homeSwerve() {
        turn.set(ControlMode.Position, 0);
        // turn.set(ControlMode.Position, getZeroPositionTicks());
    }

    public void defensiveLock(double position) {
        turn.set(ControlMode.Position, position);
    }

    /**
     * Returns a SwerveModulePosition object representing the distance and angle of the module
     * @return SwerveModulePosition object of the distance(meters) and angle(rotation2d)
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getTurnPositionAsRotation2d());
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
        return getState().angle.unaryMinus();
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
        return Helpers.General.encoderToMeters(drive.getPosition().getValueAsDouble(), this.driveWheelDiam);
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

    /**
     * This function is used to output data to the dashboard for debugging the module, typically done in the periodic method.
     */
    public void updateDashboard() {
        DashboardOld.DriveTrain.setTurnPosition(moduleName, (int) turn.getSelectedSensorPosition(0) & 0x3FF);
        DashboardOld.DriveTrain.setTurnSetpoint(moduleName, (int) turn.getClosedLoopTarget(0) & 0x3FF);
        DashboardOld.DriveTrain.setTurnPositionError(moduleName, turn.getClosedLoopError(0));
        DashboardOld.DriveTrain.setTurnVelocity(moduleName, turn.getSelectedSensorVelocity(0));
        DashboardOld.DriveTrain.setTurnZeroPosition(moduleName, getZeroPositionTicks()); 
        // Dashboard.DriveTrain.setTurnPositionErrorChange(moduleName, turn.getErrorDerivative(0));
        // Dashboard.DriveTrain.setDriveVelocity(moduleName, drive.getVelocity());
        DashboardOld.DriveTrain.setDriveDistance(moduleName, getDriveDistanceMeters());
    }
}