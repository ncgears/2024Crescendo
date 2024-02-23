
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
 * This subsystem handles managing the Arm.
 * It is responsible for moving the arm with the indexer for scoring trap and amp notes.
 */
public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;
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
  private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0);
  // private final PositionVoltage m_voltagePosition = new PositionVoltage(0);
  private CANcoder m_encoder;
  private TalonFX m_motor1;
  private State m_curState = State.STOP;
  private Double m_targetPosition = 0.0;
  
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

    m_motor1 = new TalonFX(Constants.Arm.kMotorID, Constants.Arm.canBus);
    m_motor1.setInverted(Constants.Arm.kIsInverted);
    RobotContainer.ctreConfigs.retryConfigApply(()->m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.armFXConfig));

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    m_curState = State.STOP;
    m_targetPosition = 0.0;
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
      .withPosition(14, 8);  
		if(Constants.Arm.debugDashboard) {
      ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Arm");
			ShuffleboardLayout armList = debugTab.getLayout("Arm", BuiltInLayouts.kList)
				.withSize(4,6)
				.withPosition(0,0)
				.withProperties(Map.of("Label position","LEFT"));
			armList.addString("Status", this::getColor)
				.withWidget("Single Color View");
			armList.addString("State", this::getStateName);
			armList.addNumber("Target", this::getTargetPosition);
			armList.addNumber("Position", this::getPosition);
			armList.addNumber("Absolute", this::getPositionAbsolute);
			armList.addNumber("Error", this::getPositionError);
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public String getColor() { return m_curState.getColor(); }

  public double getTargetPosition() { return m_motor1.getClosedLoopReference().getValue(); } //m_targetPosition; }
  public double getPositionError() { return m_motor1.getClosedLoopError().getValue(); }
  // public double atSetpoint() { return m_motor1.getClosedLoopError().getValue() <= Constants.Aimer.kPositionThreshold; }

  public double getPosition() {
    return m_motor1.getPosition().getValue();
  }

  public double getPositionAbsolute() {
    return m_encoder.getPosition().getValue();
  }

  public void setPosition(double position) {
    m_targetPosition = position;
    m_motor1.setControl(m_mmVoltage.withPosition(position));
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
