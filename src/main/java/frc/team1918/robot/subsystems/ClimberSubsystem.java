
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

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
  public enum LatchPosition {
    OUT(0,0,"#00FF00"), //TODO set positions
    IN(90,90,"#FF0000"); //TODO set positions
    private final int left,right;
    private final String color;
    LatchPosition(int left, int right, String color) { this.left=left; this.right=right; this.color=color; }
    public int getLeft() { return this.left; }
    public int getRight() { return this.right; }
    public String getColor() { return this.color; }
  }
  private NeutralOut m_neutral = new NeutralOut();
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private LatchPosition m_curLatchPosition = LatchPosition.OUT;
  private Servo m_leftServo = new Servo(Constants.Climber.kLeftServoID);
  private Servo m_rightServo = new Servo(Constants.Climber.kRightServoID);
  
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
    m_encoder = new CANcoder(Constants.Climber.kCANcoderID, Constants.Climber.canBus);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_encoder.getConfigurator().apply(RobotContainer.ctreConfigs.climberCCConfig));

    m_motor1 = new TalonFX(Constants.Climber.kMotorID, Constants.Climber.canBus);
    m_motor1.setInverted(Constants.Climber.kIsInverted);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.climberFXConfig));

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    climberStop();
    setLatchOut();
    Helpers.Debug.debug("Climber: Initialized");
  }
  
  @Override
  public void periodic() {
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
    driverTab.addString("Climber", this::getStateColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(16, 7);  
		if(Constants.Climber.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Climber");
      debugTab.addString("Climber", this::getStateColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      debugTab.addString("State", this::getStateName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
        debugTab.addNumber("Target", this::getTargetPosition)
        .withSize(2,2)
        .withPosition(6,0);
      debugTab.addNumber("Position", this::getPosition)
        .withSize(2,2)
        .withPosition(8,0)
        .withWidget("Text Display");
      debugTab.addNumber("Absolute", this::getPositionAbsolute)
        .withSize(2,2)
        .withPosition(10,0);
      debugTab.addNumber("Error", this::getPositionError)
        .withSize(2,2)
        .withPosition(12,0);
      debugTab.add("Climber Up", new InstantCommand(this::climberUp))
        .withSize(4, 2)
        .withPosition(0, 2)
        .withProperties(Map.of("show_type",false));  
      debugTab.add("Climber Down", new InstantCommand(this::climberDown))
        .withSize(4, 2)
        .withPosition(4, 2)
        .withProperties(Map.of("show_type",false));  
      debugTab.add("Climber Hold", new InstantCommand(this::climberHold))
        .withSize(4, 2)
        .withPosition(8, 2)  
        .withProperties(Map.of("show_type",false));  
      debugTab.add("Climber Stop", new InstantCommand(this::climberStop))
        .withSize(4, 2)
        .withPosition(12, 2)
        .withProperties(Map.of("show_type",false));  
      debugTab.addString("Latch", this::getLatchColor)
        .withSize(2, 2)
        .withPosition(0, 4)  
        .withWidget("Single Color View");
      debugTab.addString("Latch Pos", this::getLatchPostionName)
        .withSize(2,2)
        .withPosition(2,4)
        .withWidget("Text Display");
      debugTab.addNumber("Left Angle", () -> { return m_curLatchPosition.getLeft(); })
        .withSize(2,2)
        .withPosition(4,4)
        .withWidget("Text Display");
      debugTab.addNumber("Right Angle", () -> { return m_curLatchPosition.getRight(); })
        .withSize(2,2)
        .withPosition(6,4)
        .withWidget("Text Display");
      debugTab.add("Latch In", new InstantCommand(this::setLatchIn).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(0,6)
        .withProperties(Map.of("show_type",false));  
      debugTab.add("Latch Out", new InstantCommand(this::setLatchOut).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(4, 6)
        .withProperties(Map.of("show_type",false));  
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getStateColor() { return m_curState.getColor(); }
  public LatchPosition getLatchPosition() { return m_curLatchPosition; }
  public String getLatchPostionName() { return m_curLatchPosition.toString(); }
  public String getLatchColor() { return m_curLatchPosition.getColor(); }

  private void setLatchPosition(LatchPosition pos) {
    m_curLatchPosition = pos;
    m_leftServo.setAngle(pos.getLeft());
    m_rightServo.setAngle(pos.getRight());
  }
  public void setLatchOut() {
    setLatchPosition(LatchPosition.OUT);
    Helpers.Debug.debug("Climber: Latch Out");
  }
  public void setLatchIn() {
    setLatchPosition(LatchPosition.IN);
    Helpers.Debug.debug("Climber: Latch In");
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

  public void climberUp() {
    m_curState = State.UP;
    Helpers.Debug.debug("Climber: Up");
  }
  public void climberDown() {
    m_curState = State.DOWN;
    Helpers.Debug.debug("Climber: Down");
  }
  public void climberHold() {
    m_motor1.setControl(new StaticBrake());
    m_curState = State.HOLD;
    Helpers.Debug.debug("Climber: Hold");
  }
  public void climberStop() {
    m_motor1.setControl(m_neutral);
    m_curState = State.STOP;
    Helpers.Debug.debug("Climber: Stop");
  }
}
