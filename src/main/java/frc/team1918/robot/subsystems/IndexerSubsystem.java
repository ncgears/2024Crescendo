
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    UP(Constants.Dashboard.Colors.GREEN),
    DOWN(Constants.Dashboard.Colors.RED),
    STOP(Constants.Dashboard.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private WPI_TalonSRX m_motor1;
  private State m_curState = State.STOP;
  private boolean m_hasnote = false;
  private DigitalInput m_beambreak = new DigitalInput(Constants.Indexer.kBeamBreakID);
  private String m_overrideColor = null;
  
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
    m_motor1 = new WPI_TalonSRX(Constants.Indexer.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    m_motor1.setNeutralMode(Constants.Indexer.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Indexer.kIsInverted);
    init();
    createDashboards();
  }

  public final Trigger isFull = new Trigger(this::hasNote);

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

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout indexerList = systemTab.getLayout("Indexer", BuiltInLayouts.kList)
      .withSize(4,3)
      .withPosition(8,0)
      .withProperties(Map.of("Label position","LEFT"));
    indexerList.addString("Status", this::getColor)
      .withWidget("Single Color View");
    indexerList.addString("State", this::getStateName);
    indexerList.addBoolean("Has Note", this::hasNote);

    if(Constants.Indexer.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgIndexerList = debugTab.getLayout("Indexer", BuiltInLayouts.kList)
        .withSize(4,4)
        .withPosition(12,0)
        .withProperties(Map.of("Label position","LEFT"));
      dbgIndexerList.addString("Status", this::getColor)
        .withWidget("Single Color View");
      dbgIndexerList.addString("State", this::getStateName);
      dbgIndexerList.addBoolean("Has Note", this::hasNote);
      dbgIndexerList.add("Debug: Note Toggle", new InstantCommand(this::toggleFull).ignoringDisable(true))
        .withProperties(Map.of("show_type",false));  
    }
  }

  /**
   * Sets the speed of the Indexer
   * @param speed The speed of the Indexer in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return (m_overrideColor != null) ? m_overrideColor : m_curState.getColor(); }
  public void setColor(String color) { m_overrideColor = color; }

  public State getDirection() { return m_curState; }
  public String getDirectionName() { return m_curState.toString(); }

  public void indexerUp() {
    m_curState = State.UP;
    Helpers.Debug.debug("Intake: In");
    setSpeedPercent(Constants.Indexer.kSpeed);
  }

  public void indexerDown() {
    m_curState = State.DOWN;
    Helpers.Debug.debug("Intake: Out");
    setSpeedPercent(-Constants.Indexer.kSpeed);
  }

  public void indexerStop() {
    m_curState = State.STOP;
    Helpers.Debug.debug("Intake: Stop");
    setSpeedPercent(0.0);
  }

  public boolean hasNote() {
    // return m_hasnote; //TODO: switch to beambreak
    return getBeamBreak();
  }

  private boolean getBeamBreak() {
    return m_beambreak.get();
  }

  private void toggleFull() {
    m_hasnote = !m_hasnote;
  }

}
