
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.Servo;
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
 * This subsystem handles managing the Climber.
 * It is responsible for extending and retracting the elevator/climber.
 */
public class ClimberSubsystem extends SubsystemBase {
	private static ClimberSubsystem instance;
  //private and public variables defined here
  public enum State {
    UP(Constants.Dashboard.Colors.GREEN),
    DOWN(Constants.Dashboard.Colors.RED),
    HOLD(Constants.Dashboard.Colors.ORANGE),
    STOP(Constants.Dashboard.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  public enum LatchPosition {
    OUT(0,0,Constants.Dashboard.Colors.GREEN), //TODO set positions
    IN(90,90,Constants.Dashboard.Colors.RED); //TODO set positions
    private final int left,right;
    private final String color;
    LatchPosition(int left, int right, String color) { this.left=left; this.right=right; this.color=color; }
    public int getLeft() { return this.left; }
    public int getRight() { return this.right; }
    public String getColor() { return this.color; }
  }
  public enum Position {
    TOP(Constants.Climber.Positions.kTop),
    BOTTOM(Constants.Climber.Positions.kBottom),
    TOPCLIMB(Constants.Climber.Positions.kTopHookClimb),
    MIDCLEAR(Constants.Climber.Positions.kMidHookClear),
    LATCHCLIMB(Constants.Climber.Positions.kLatchClimb),
    LATCHCLEAR(Constants.Climber.Positions.kLatchClear);
    private final double position;
    Position(double position) { this.position = position; }
    public double getAngularPositionRotations() { return this.position; }
  }
  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  private final DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Position m_targetPosition = Position.BOTTOM;
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
    m_targetPosition = Position.BOTTOM;
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
      .withPosition(18, 7);  

    ShuffleboardTab systemTab = Shuffleboard.getTab("System");
    ShuffleboardLayout climberList = systemTab.getLayout("Climber", BuiltInLayouts.kList)
      .withSize(4,6)
      .withPosition(16,0)
      .withProperties(Map.of("Label position","LEFT"));
    climberList.addString("Status", this::getStateColor)
      .withWidget("Single Color View");
    climberList.addString("State", this::getStateName);
    climberList.addString("Target", this::getTargetPositionName);
    climberList.addNumber("Target Pos", this::getTargetPosition);
    climberList.addNumber("Position", this::getPosition);
    climberList.addNumber("Absolute", this::getPositionAbsolute);
    climberList.addNumber("Error", this::getPositionError);
    climberList.addBoolean("Rev Lim", this::getReverseLimit);

    ShuffleboardLayout latchList = systemTab.getLayout("Latch", BuiltInLayouts.kList)
      .withSize(4,4)
      .withPosition(20,0)
      .withProperties(Map.of("Label position","LEFT"));
    latchList.addString("Status", this::getLatchColor)
      .withWidget("Single Color View");
    latchList.addString("State", this::getLatchPostionName);
    latchList.addNumber("Left Angle", () -> { return m_curLatchPosition.getLeft(); });
    latchList.addNumber("Right Angle", () -> { return m_curLatchPosition.getRight(); });

    if(Constants.Climber.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
			ShuffleboardLayout dbgClimberList = debugTab.getLayout("Climber", BuiltInLayouts.kList)
				.withSize(4,10)
				.withPosition(16,0)
				.withProperties(Map.of("Label position","LEFT"));
      dbgClimberList.addString("Status", this::getStateColor)
        .withWidget("Single Color View");
      dbgClimberList.addString("State", this::getStateName);
      dbgClimberList.addNumber("Target", this::getTargetPosition);
      dbgClimberList.addNumber("Position", this::getPosition);
      dbgClimberList.addNumber("Absolute", this::getPositionAbsolute);
      dbgClimberList.addNumber("Error", this::getPositionError);
      dbgClimberList.addBoolean("Rev Lim", this::getReverseLimit);
      dbgClimberList.add("Climber Up", new InstantCommand(this::climberUp))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Down", new InstantCommand(this::climberDown))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Hold", new InstantCommand(this::climberHold))
        .withProperties(Map.of("show_type",false));  
      dbgClimberList.add("Climber Stop", new InstantCommand(this::climberStop))
        .withProperties(Map.of("show_type",false));  

      ShuffleboardLayout dbgLatchList = debugTab.getLayout("Latch", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(12,4)
				.withProperties(Map.of("Label position","LEFT"));
			dbgLatchList.addString("Status", this::getLatchColor)
				.withWidget("Single Color View");
			dbgLatchList.addString("State", this::getLatchPostionName);
			dbgLatchList.addNumber("Left Angle", () -> { return m_curLatchPosition.getLeft(); });
			dbgLatchList.addNumber("Right Angle", () -> { return m_curLatchPosition.getRight(); });
      dbgLatchList.add("Latch In", new InstantCommand(this::setLatchIn).ignoringDisable(true))
        .withProperties(Map.of("show_type",false)); 
      dbgLatchList.add("Latch Out", new InstantCommand(this::setLatchOut).ignoringDisable(true))
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

  public String getTargetPositionName() { return m_targetPosition.toString(); }
  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }
  // public double atSetpoint() { return m_motor1.getClosedLoopError().getValue() <= Constants.Aimer.kPositionThreshold; }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public void setPosition(Position position) {
    m_motor1.setControl(m_mmVoltage.withPosition(position.getAngularPositionRotations()));
  }

  public boolean getForwardLimit() {
    //if using NormallyOpen, this should be ForwardLimitValue.ClosedToGround
    return m_motor1.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  public boolean getReverseLimit() {
    //if using NormallyOpen, this should be ReverseLimitValue.ClosedToGround
    return m_motor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public void climberMove(double power) {
    if(power>0) {
      if(m_curState != State.UP) {
        Helpers.Debug.debug("Climber: Up ("+power+")");
        m_curState = State.UP;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else if(power<0) {
      if(m_curState != State.DOWN) {
        Helpers.Debug.debug("Climber: Down ("+power+")");
        m_curState = State.DOWN;
      }
      m_motor1.setControl(m_DutyCycle.withOutput(power));
    } else { //0 power
      if(m_curState != State.HOLD && m_curState != State.STOP) {
        m_motor1.setControl(m_brake);
        m_curState = State.HOLD;
        Helpers.Debug.debug("Climber: Hold");
      }
    }
  }

  public void climberUp() {
    m_curState = State.UP;
    m_motor1.setControl(m_DutyCycle);
    Helpers.Debug.debug("Climber: Up");
  }
  public void climberDown() {
    m_curState = State.DOWN;
    Helpers.Debug.debug("Climber: Down");
  }
  public void climberHold() {
    m_motor1.setControl(m_brake);
    if(m_curState != State.HOLD) {
      m_curState = State.HOLD;
      Helpers.Debug.debug("Climber: Hold");
    }
  }
  public void climberStop() {
    m_motor1.setControl(m_neutral);
    if(m_curState != State.HOLD) {
      m_curState = State.STOP;
      Helpers.Debug.debug("Climber: Stop");
    }
  }
}
