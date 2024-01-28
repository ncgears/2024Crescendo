
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
 * This subsystem handles managing the Indexer.
 * It is responsible for holding a note until the shooter requests it, and for advancing notes into the shooter.
 */
public class IndexerSubsystem extends SubsystemBase {
	private static IndexerSubsystem instance;
  //private and public variables defined here
  public enum State {
    FWD("#FFA500"),
    LOADED("#00FF00"),
    STOP("#000000");
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private WPI_TalonSRX m_motor1;
  private State m_curState = State.STOP;
  
  /**
	 * Returns the instance of the IndexerSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return IndexerSubsystem instance
	 */
  public static IndexerSubsystem getInstance() {
		if (instance == null)
			instance = new IndexerSubsystem();
		return instance;
	}
  
  public IndexerSubsystem() {
    //initialize values for private and public variables, etc.
    m_motor1 = new WPI_TalonSRX(Constants.Intake.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    m_motor1.setNeutralMode(Constants.Intake.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Intake.kIsInverted);
    init();
    createDashboards();
  }
   
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_motor1.set(ControlMode.PercentOutput,0);
    m_curState = State.STOP;
    Helpers.Debug.debug("Indexer: Initialized");
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
    driverTab.addString("Indexer", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(10, 7);  
		if(Constants.Intake.debugDashboard) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Debug: Indexer");
      intakeTab.addString("Indexer", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      intakeTab.addString("State", this::getStateName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
      intakeTab.add("Update State", new InstantCommand(this::updateState))
        .withSize(4, 2)
        .withPosition(6, 0);  
    }
  }

  /**
   * Sets the speed of the Intake
   * @param speed The speed of the Intake in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }

  public void updateState() {
    
  }
}
