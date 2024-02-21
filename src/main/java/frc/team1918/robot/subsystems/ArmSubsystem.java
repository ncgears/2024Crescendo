
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

/**
 * This subsystem handles managing the Arm.
 * It is responsible for moving the arm with the indexer for scoring trap and amp notes.
 */
public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;
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
  private CANcoder m_encoder;
  private TalonFX m_motor1New;
  private WPI_TalonSRX m_motor1;
  private State m_curState = State.STOP;
  
  /**
	 * Returns the instance of the ArmSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ArmSubsystem instance
	 */
  public static ArmSubsystem getInstance() {
		if (instance == null)
			instance = new ArmSubsystem();
		return instance;
	}
  
  public ArmSubsystem() {
    //initialize values for private and public variables, etc.
    m_encoder = new CANcoder(Constants.Arm.kCANcoderID, Constants.Arm.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.armCCConfig));

    m_motor1 = new WPI_TalonSRX(Constants.Arm.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    // m_motor1.setNeutralMode(Constants.Arm.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Arm.kIsInverted);
    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_motor1.set(ControlMode.PercentOutput,0);
    m_curState = State.STOP;
    Helpers.Debug.debug("Arm: Initialized");
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
    driverTab.addString("Arm", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(14, 7);  
		if(Constants.Arm.debugDashboard) {
      ShuffleboardTab armTab = Shuffleboard.getTab("DBG:Arm");
      armTab.addString("Arm", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      armTab.addString("State", this::getStateName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
      armTab.add("Arm Up", new InstantCommand(this::armUp))
        .withSize(4, 2)
        .withPosition(0, 2);  
      armTab.add("Arm Down", new InstantCommand(this::armDown))
        .withSize(4, 2)
        .withPosition(4, 2);  
      armTab.add("Arm Hold", new InstantCommand(this::armHold))
        .withSize(4, 2)
        .withPosition(8, 2);  
      armTab.add("Arm Stop", new InstantCommand(this::armStop))
        .withSize(4, 2)
        .withPosition(12, 2);  
    }
  }

  /**
   * Sets the speed of the Arm
   * @param speed The speed of the Arm in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }

  public void updateState() {
    
  }

  public void armUp() {
    m_curState = State.UP;
    Helpers.Debug.debug("Arm: Up");
  }
  public void armDown() {
    m_curState = State.DOWN;
    Helpers.Debug.debug("Arm: Down");
  }
  public void armHold() {
    m_curState = State.HOLD;
    Helpers.Debug.debug("Arm: Hold");
  }
  public void armStop() {
    m_curState = State.STOP;
    Helpers.Debug.debug("Arm: Stop");
  }
}
