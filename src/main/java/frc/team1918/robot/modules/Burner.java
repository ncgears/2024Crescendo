
package frc.team1918.robot.modules;
//Talon SRX/Talon FX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

//Team 1918
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.utils.PIDGains;
import frc.team1918.robot.utils.TalonConstants;

public class Burner {
    private WPI_TalonSRX m_motor;
    private String m_moduleName;
    public enum BurnerPositions {ZERO, HOME, HOT, TOP, COLD, BOTTOM};
    public BurnerPositions currentPosition = BurnerPositions.HOME;

 	/**
	 * 1918 HotPlate Module v2023.1 - This spatula module uses a TalonSRX with 775, 550, or Bag motor on a Versa Planetary to serve scoring pieces
     * The module uses a Versa-Planetary Encoder Stage for positioning data.
     * There is a high limit and low limit switch, connected to the Talon to limit the mechanical travel
	 * @param name This is the name of this spatula module (ie. "HotPlate")
     * @param moduleConstants This is a HotPlateConstants object containing the data for this module
	 */
    public Burner(String name, TalonConstants moduleConstants, PIDGains moduleGains){
        m_moduleName = name;
        m_motor = new WPI_TalonSRX(moduleConstants.MotorID);

        m_motor.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
        m_motor.set(ControlMode.PercentOutput, 0); //Set controller to stopped
        m_motor.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        m_motor.configSelectedFeedbackSensor(  FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
            Constants.Global.kPidIndex,				// PID Slot for Source [0, 1]
            Constants.Global.kTimeoutMs);				// Configuration Timeout

        //m_motor.configFeedbackNotContinuous(moduleConstants.SensorNonContinuous, 0); //Disable continuous feedback tracking (so 0 and 1024 are effectively one and the same)
        m_motor.configNeutralDeadband(moduleGains.kNeutralDeadband, Constants.Global.kTimeoutMs); //Adjust deadband to 0.1% from default of 4%

        //if the sensor phase is wrong, plot will go opposite direction of target, need to fix before setting inverted
        m_motor.setSensorPhase(moduleConstants.SensorPhase); //set the sensor phase based on the constants setting for this module
        //once sensor phase is correct, set inverted so "forward" is green lights, "reverse" is red lights
        m_motor.setInverted(moduleConstants.IsInverted); //set the motor direction based on the constants setting for this module

        /* Set relevant frame periods to be at least as fast as periodic rate */
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Global.kTimeoutMs);
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Global.kTimeoutMs);        

        /* Set peak and nominal outputs */
        m_motor.configNominalOutputForward(0, Constants.Global.kTimeoutMs);
        m_motor.configNominalOutputReverse(0, Constants.Global.kTimeoutMs);
        m_motor.configPeakOutputForward(moduleGains.kPeakOutput, Constants.Global.kTimeoutMs);
        m_motor.configPeakOutputReverse(-moduleGains.kPeakOutput, Constants.Global.kTimeoutMs);

        /* Set Motion Magic gains in slot0 */
        m_motor.selectProfileSlot(Constants.Global.kPidProfileSlotIndex, Constants.Global.kPidIndex);
        m_motor.config_kF(Constants.Global.kPidProfileSlotIndex, moduleGains.kF, Constants.Global.kTimeoutMs);
        m_motor.config_kP(Constants.Global.kPidProfileSlotIndex, moduleGains.kP, Constants.Global.kTimeoutMs);
        m_motor.config_kI(Constants.Global.kPidProfileSlotIndex, moduleGains.kI, Constants.Global.kTimeoutMs);
        m_motor.config_kD(Constants.Global.kPidProfileSlotIndex, moduleGains.kD, Constants.Global.kTimeoutMs);
        //m_motor.config_IntegralZone(0, m_kIZone);
        m_motor.configAllowableClosedloopError(Constants.Global.kPidIndex, moduleConstants.AllowedError); 

        /* Set acceleration and cruise velocity */
        m_motor.configMotionCruiseVelocity(moduleGains.kCruise, Constants.Global.kTimeoutMs);
        m_motor.configMotionAcceleration(moduleGains.kAccel, Constants.Global.kTimeoutMs);
        m_motor.configMotionSCurveStrength(moduleGains.kSCurve, Constants.Global.kTimeoutMs);

        /* Configure limit switches */
        // NormallyOpen is default, so we really only need to change this when we have connected NC switches
        m_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.Global.kTimeoutMs);
        m_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.Global.kTimeoutMs);

        /* Zero the sensor when reverse limit triggered */
        m_motor.configClearPositionOnLimitF(false, Constants.Global.kTimeoutMs);
        m_motor.configClearPositionOnLimitR(true, Constants.Global.kTimeoutMs);

        /* Zero the sensor on robot boot */
        // m_motor.setSelectedSensorPosition(0); //reset the talon encoder counter to 0 so we dont carry over a large error from a previous testing
    }

    /**
     *  Returns the current position of the spatula in encoder counts
     * @return The current encoder position of the spatula
     */
    public double getPositionAbsolute() {
        return m_motor.getSelectedSensorPosition(Constants.Global.kPidIndex);
    }

    /**
     * Sets the percent output of the controller to a given speed
     * @param speed (double) speed of motor controller
     */
    public void setSpeed(double speed) {
        m_motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the brake mode for the spatula (to allow it to move freely)
     * @param brake Boolean indicating if the brake mode should be set to brake (true) or coast (false)
     */
    public void setBrakeMode(String device, boolean brake) {
        if (brake) {
            m_motor.setNeutralMode(NeutralMode.Brake);
        } else {
            m_motor.setNeutralMode(NeutralMode.Coast);
        }
    }

    public String getModuleName() {
        return m_moduleName;
    }

    public void moveTo(BurnerPositions position) {
        currentPosition = position;
        switch (position) {
            case HOME:
                m_motor.set(ControlMode.MotionMagic, Constants.Stove.Burner.Positions.home);
                break;
            case TOP:
            case HOT:
                m_motor.set(ControlMode.MotionMagic, Constants.Stove.Burner.Positions.top);
                break;
            case BOTTOM:
            case COLD:
                m_motor.set(ControlMode.MotionMagic, Constants.Stove.Burner.Positions.bottom);
                break;
            default:
                Dashboard.Burner.setPositionName("Unknown");
        }
    }

    /**
     * This function sets the sensor position to zero
     */
    public void setZeroPos() {
        m_motor.setSelectedSensorPosition(0);
    }

    /**
     * This function is used to return the state of the reverse limit switch
     * @return (boolean) state of the limit switch
     */
    public boolean isRevLimit() {
        return m_motor.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * This function is used to return the state of the forward limit switch
     * @return (boolean) state of the limit switch
     */
    public boolean isFwdLimit() {
        return m_motor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * This function is used to output data to the dashboard for debugging the module, typically done in the periodic method.
     */
    public void updateDashboard() {
        Dashboard.Burner.setPositionName(currentPosition.toString());
        if(currentPosition != BurnerPositions.ZERO) {
            Dashboard.Burner.setPosition((int) m_motor.getSelectedSensorPosition(Constants.Global.kPidIndex));
            Dashboard.Burner.setTarget((int) m_motor.getClosedLoopTarget(Constants.Global.kPidIndex));
            Dashboard.Burner.setError((int) m_motor.getClosedLoopError(Constants.Global.kPidIndex));
            Dashboard.Burner.setFLimit((boolean) isFwdLimit());
            Dashboard.Burner.setRLimit((boolean) isRevLimit());
        }
        Dashboard.Burner.setSpeed(m_motor.getMotorOutputPercent());
    }

}