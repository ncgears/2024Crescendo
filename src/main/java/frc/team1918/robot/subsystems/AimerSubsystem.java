
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

  private WPI_TalonSRX m_motor1;
  private State m_curState = State.STOP;
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
    m_motor1 = new WPI_TalonSRX(Constants.Intake.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    m_motor1.setNeutralMode(Constants.Intake.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Intake.kIsInverted);
    m_curState = State.STOP;
    createDashboards();
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
    driverTab.addString("Aimer", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  
		if(Constants.Intake.debugDashboard) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Debug: Aimer");
      intakeTab.addString("Aimer", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      intakeTab.addString("State", () -> getState().toString())
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
  public String getColor() { return m_curState.getColor(); }

  public void updateSetpoint() {
    if(!m_suppressTracking) { //allowed to track
      //determine the proper setpoint from vision and set it as the closed loop target
      // RobotContainer.vision.getTarget()
    }
  }

  public void updateState() {
    //TODO: If the closed loop error is under threshold, then consider the aimer "READY"
    if(false) m_curState = State.READY;
  }

  public void setPosition(double position) {
    m_motor1.set(ControlMode.Position, position);
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
