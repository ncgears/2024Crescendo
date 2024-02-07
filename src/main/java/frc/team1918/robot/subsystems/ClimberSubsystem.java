
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;

/**
 * This subsystem handles managing the Climber.
 * It is responsible for extending and retracting the elevator/climber.
 */
public class ClimberSubsystem extends SubsystemBase {
	private static ClimberSubsystem instance;
  //private and public variables defined here
  public enum State {
    UP("#00FF00"),
    DOWN("#FF0000"),
    HOLD("#FFA500"),
    STOP("#000000");
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private WPI_TalonSRX m_motor1;
  private State m_curState = State.STOP;
  
  /**
	 * Returns the instance of the ClimberSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ClimberSubsystem instance
	 */
  public static ClimberSubsystem getInstance() {
		if (instance == null)
			instance = new ClimberSubsystem();
		return instance;
	}
  
  public ClimberSubsystem() {
    //initialize values for private and public variables, etc.
    m_motor1 = new WPI_TalonSRX(Constants.Climber.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    // m_motor1.setNeutralMode(Constants.Climber.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Climber.kIsInverted);
    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_motor1.set(ControlMode.PercentOutput,0);
    m_curState = State.STOP;
    Helpers.Debug.debug("Climber: Initialized");
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
    driverTab.addString("Climber", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(16, 7);  
		if(Constants.Climber.debugDashboard) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("DBG:Climber");
      climberTab.addString("Climber", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      climberTab.addString("State", this::getStateName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
      climberTab.add("Climber Up", new InstantCommand(this::climberUp))
        .withSize(4, 2)
        .withPosition(0, 2);  
      climberTab.add("Climber Down", new InstantCommand(this::climberDown))
        .withSize(4, 2)
        .withPosition(4, 2);  
      climberTab.add("Climber Hold", new InstantCommand(this::climberHold))
        .withSize(4, 2)
        .withPosition(8, 2);  
      climberTab.add("Climber Stop", new InstantCommand(this::climberStop))
        .withSize(4, 2)
        .withPosition(12, 2);  
    }
  }

  /**
   * Sets the speed of the Climber
   * @param speed The speed of the Climber in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }

  public void updateState() {
    
  }

  public void climberUp() {
    m_curState = State.UP;
    Helpers.Debug.debug("Climber: Up");
  }
  public void climberDown() {
    m_curState = State.DOWN;
    Helpers.Debug.debug("Climber: Down");
  }
  public void climberHold() {
    m_curState = State.HOLD;
    Helpers.Debug.debug("Climber: Hold");
  }
  public void climberStop() {
    m_curState = State.STOP;
    Helpers.Debug.debug("Climber: Stop");
  }
}
