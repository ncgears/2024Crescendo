
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
 * This subsystem handles managing the Intake.
 * It is responsible for running the intake using information from the indexer about whether we have a note.
 */
public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;
  //private and public variables defined here
  public enum Direction {
    IN("#00FF00"),
    OUT("#FF0000"),
    STOP("#000000");
    private final String color;
    Direction(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private WPI_TalonSRX m_motor1;
  private Direction m_curDirection = Direction.STOP;
 
  /**
	 * Returns the instance of the IntakeSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return IntakeSubsystem instance
	 */
  public static IntakeSubsystem getInstance() {
		if (instance == null)
			instance = new IntakeSubsystem();
		return instance;
	}
  
  public IntakeSubsystem() {
    //initialize values for private and public variables, etc.
    m_motor1 = new WPI_TalonSRX(Constants.Intake.kMotorID);
    m_motor1.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff from carrying over
    m_motor1.set(ControlMode.PercentOutput, 0); //Set controller to disabled
    m_motor1.setNeutralMode(Constants.Intake.kNeutralMode); //Set controller to brake mode  
    m_motor1.setInverted(Constants.Intake.kIsInverted);
    m_curDirection = Direction.STOP;
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
    driverTab.addString("Intake", this::getColor)
      .withSize(2, 2)
      .withWidget("Single Color View")
      .withPosition(8, 7);  
		if(Constants.Intake.debugDashboard) {
      ShuffleboardTab intakeTab = Shuffleboard.getTab("Debug: Intake");
      intakeTab.addString("Intake", this::getColor)
        .withSize(2, 2)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      intakeTab.addString("Direction", this::getDirectionName)
        .withSize(4,2)
        .withPosition(2,0)
        .withWidget("Text Display");
      intakeTab.add("Intake In", new InstantCommand(this::intakeIn))
        .withSize(4, 2)
        .withPosition(6, 0);  
      intakeTab.add("Intake Out", new InstantCommand(this::intakeOut))
        .withSize(4, 2)
        .withPosition(10, 0);  
      intakeTab.add("Intake Stop", new InstantCommand(this::intakeStop))
        .withSize(4, 2)
        .withPosition(14, 0);  
    }
  }

  /**
   * Sets the speed of the Intake
   * @param speed The speed of the Intake in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public Direction getDirection() { return m_curDirection; }
  public String getDirectionName() { return m_curDirection.toString(); }
  public String getColor() { return m_curDirection.getColor(); }

  public void intakeIn() {
    m_curDirection = Direction.IN;
    Helpers.Debug.debug("Intake: In");
    setSpeedPercent(Constants.Intake.kSpeed);
}

  public void intakeOut() {
    m_curDirection = Direction.OUT;
    Helpers.Debug.debug("Intake: Out");
    setSpeedPercent(-Constants.Intake.kSpeed);
  }

  public void intakeStop() {
    m_curDirection = Direction.STOP;
    Helpers.Debug.debug("Intake: Stop");
    setSpeedPercent(0.0);
  }
}
