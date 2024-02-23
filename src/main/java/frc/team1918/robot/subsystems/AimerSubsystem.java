
package frc.team1918.robot.subsystems;

import java.util.Map;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

/**
 * This subsystem handles managing the Aimer.
 * It is responsible for adjusting the vertical aim of the shooter using tracking data from vision.
 */
public class AimerSubsystem extends SubsystemBase {
	private static AimerSubsystem instance;
  //private and public variables defined here
  public enum State {
    READY(Constants.Dashboard.Colors.GREEN),
    TRACKING(Constants.Dashboard.Colors.ORANGE),
    ERROR(Constants.Dashboard.Colors.RED),
    STOP(Constants.Dashboard.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  // private final PositionVoltage m_voltagePosition = new PositionVoltage(0);

  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Double m_targetPosition = 0.0;
  private boolean m_suppressTracking = false; //do not allow tracking while true
  private DoubleSubscriber new_position_sub;
  private double new_position = 0.0;
  
  /**
	 * Returns the instance of the AimerSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return AimerSubsystem instance
	 */
  public static AimerSubsystem getInstance() {
		if (instance == null)
			instance = new AimerSubsystem();
		return instance;
	}

  public AimerSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(Constants.Aimer.kCANcoderID, Constants.Aimer.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.aimerCCConfig));

    m_motor1 = new TalonFX(Constants.Aimer.kMotorID, Constants.Aimer.canBus);
    m_motor1.setInverted(Constants.Aimer.kIsInverted);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.aimerFXConfig));
    // Helpers.Debug.debug("Could not initialize aimer motor, error: " + status1.toString());

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    Helpers.Debug.debug("Aimer: Initialized");
    m_targetPosition = 0.0;
  }

  @Override
  public void periodic() {
    new_position = new_position_sub.get(0.0);
    updateSetpoint();
    // updateState();
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Aimer", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  
		if(Constants.Aimer.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Aimer");
			ShuffleboardLayout aimerList = debugTab.getLayout("Aimer", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(0,0)
				.withProperties(Map.of("Label position","LEFT"));
			aimerList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			aimerList.addString("State", this::getStateName);
			aimerList.addNumber("Target", this::getTargetPosition);
			aimerList.addNumber("Position", this::getPosition);
			aimerList.addNumber("Absolute", this::getPositionAbsolute);
			aimerList.addNumber("Error", this::getPositionError);
      aimerList.addBoolean("Rev Lim", this::getReverseLimit);
      aimerList.addBoolean("Fwd Lim", this::getReverseLimit);
      aimerList.add("Set Zero", new InstantCommand(this::setZero).ignoringDisable(true))
        .withProperties(Map.of("show_type",false));  
      debugTab.addNumber("Target Position", this::getNewPosition)
        .withSize(4,2)
        .withPosition(4,0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min_value",0,"max_value",0.125,"divisions",10));

      new_position_sub = NetworkTableInstance.getDefault().getDoubleTopic("/Shuffleboard/DBG:Aimer/Target Position").subscribe(0.0);
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }
  public double getNewPosition() { return Helpers.General.roundDouble(new_position,3); }

  public void updateSetpoint() {
    if(!m_suppressTracking) { //allowed to track
      //determine the proper setpoint from vision and set it as the closed loop target
      // RobotContainer.vision.getTarget()
      if(new_position != m_targetPosition) setPosition(new_position);
    } else {
      setPosition(m_targetPosition);
    }
  }

  public void updateState() {
    //TODO: If the closed loop error is under threshold, then consider the aimer "READY"
    if(true) m_curState = State.READY;
  }

  public void setPosition(double position) {
    // m_motor1.setControl(m_voltagePosition.withPosition(position));
    m_targetPosition = position;
    m_motor1.setControl(m_mmVoltage.withPosition(position));
  }

  public void setZero() {
    m_motor1.setPosition(0.0);
  }

  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); } //m_targetPosition; }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }
  // public double atSetpoint() { return m_motor1.getClosedLoopError().getValue() <= Constants.Aimer.kPositionThreshold; }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public boolean getForwardLimit() {
    //if using NormallyOpen, this should be ForwardLimitValue.ClosedToGround
    return m_motor1.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getReverseLimit() {
    //if using NormallyOpen, this should be ReverseLimitValue.ClosedToGround
    return m_motor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public void aimerStopAndStow() {
    m_suppressTracking = true;
    m_curState = State.STOP;
    Helpers.Debug.debug("Aimer: Stop and Stow");
    setPosition(Constants.Aimer.kStowPosition);
  }

  public void aimerStartTracking() {
    m_suppressTracking = false;
    m_curState = State.TRACKING;
    Helpers.Debug.debug("Aimer: Start Tracking");
  }

}
