
package frc.team1918.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    READY("#00FF00"),
    TRACKING("#FFA500"),
    ERROR("#FF0000"),
    STOP("#000000");
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private PositionVoltage m_voltagePosition = new PositionVoltage(0,0,true,0,0,false, false, false);
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Double m_targetPosition = 0.0;
  private boolean m_suppressTracking = false; //do not allow tracking while true
  
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
    m_motor1 = new TalonFX(Constants.Aimer.kMotorID, Constants.Aimer.canBus);
    m_motor1.setInverted(Constants.Aimer.kIsInverted);
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.aimerFXConfig);
      if (status1.isOK()) break;
    }
    if(!status1.isOK()) {
      Helpers.Debug.debug("Could not initialize aimer motor, error: " + status1.toString());
    }

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
    // updateState();
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   builder.setSmartDashboardType("Number Slider");
  //   builder.setActuator(true);
  //   builder.addDoubleProperty("Target Speed", this::getTargetSpeed, this::setSpeedPercent);
  //   builder.addDoubleProperty("Current Speed", this::getSpeedPercent, null);
  // }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Aimer", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  
		if(Constants.Aimer.debugDashboard) {
      ShuffleboardTab aimerTab = Shuffleboard.getTab("DBG:Aimer");
      aimerTab.addString("Aimer", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      aimerTab.addString("State", this::getStateName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
      aimerTab.add("Update State", new InstantCommand(this::updateState))
        .withSize(4, 2)
        .withPosition(6, 0);
      aimerTab.addNumber("Target", this::getTargetPosition)
        .withSize(2,2)
        .withPosition(0,2);
      aimerTab.addNumber("Position", this::getPosition)
        .withSize(2,2)
        .withPosition(2,2);
      aimerTab.addNumber("Absolute", this::getPositionAbsolute)
        .withSize(2,2)
        .withPosition(4,2);
      aimerTab.addBoolean("Rev Lim", this::getReverseLimit)
        .withSize(2,2)
        .withPosition(6,2);    
      aimerTab.addBoolean("Fwd Lim", this::getForwardLimit)
        .withSize(2,2)
        .withPosition(8,2);    
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }

  public void updateSetpoint() {
    if(!m_suppressTracking) { //allowed to track
      //determine the proper setpoint from vision and set it as the closed loop target
      // RobotContainer.vision.getTarget()
    } else {
      setPosition(m_targetPosition);
    }
  }

  public void updateState() {
    //TODO: If the closed loop error is under threshold, then consider the aimer "READY"
    if(true) m_curState = State.READY;
  }

  public void setPosition(double position) {
    m_motor1.setControl(m_voltagePosition.withPosition(position));
  }

  public double getTargetPosition() { return m_targetPosition; }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_motor1.getPosition().getValue(); //TODO this should be from a through bore. Maybe we put a limit in to store the offset when limit is hit?
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
