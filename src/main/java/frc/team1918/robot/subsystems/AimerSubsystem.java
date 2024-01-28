
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

/**
 * This subsystem handles managing the Aimer.
 * It is responsible for adjusting the vertical aim of the shooter using tracking data from vision.
 */
public class AimerSubsystem extends SubsystemBase {
	private static AimerSubsystem instance;
  //private and public variables defined here
  public enum State {
    READY,
    TRACKING,
    ERROR,
    STOP;
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
    driverTab.addString("Aimer", this::getDirectionColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(12, 7);  
		if(Constants.Intake.debugDashboard) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Debug: Intake");
      intakeTab.addString("State", () -> getState().toString())
        .withSize(4,2)
        .withPosition(0,0)
        .withWidget("Text Display");
      intakeTab.add("Update State", new InstantCommand(this::updateState))
        .withSize(4, 2)
        .withPosition(4, 0);  
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
  public String getDirectionColor() { 
    switch (m_curState) {
      case READY: return "#00FF00";
      case TRACKING: return "#FFA500";
      case ERROR: return "#FF0000";
      case STOP:
      default: return "#000000";
    }
  }

  public void updateState() {
    //TODO: If the closed loop error is under threshold, then consider the aimer "READY"
    if(true) m_curState = State.READY;
  }

  public void moveTo(double position) {
    m_motor1.set(ControlMode.Position, position);
  }

  public void aimerStopAndStow() {
    m_suppressTracking = true;
    moveTo(Constants.Aimer.kStowPosition);
  }

  public void aimerStartTracking() {
    m_suppressTracking = false;
  }

}
