
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
// import frc.team1918.robot.utils.TunableNumber;
import frc.team1918.robot.RobotContainer;

/**
 * The Template Subsystem handles getting and managing the Template.
 * It is responsible for doing some stuff.
 */
public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;
  //private and public variables defined here
  public double target_speed = 0.0;
  private TalonSRX m_motor1;
  
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
    m_motor1 = new TalonSRX(Constants.Intake.kMotorID);
    createDashboards();
  }
  
  @Override
  public void periodic() {
    updateDashboard();
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
		// if(Constants.Intake.debugDashboard) {
    //   ShuffleboardTab intakeTab = Shuffleboard.getTab("Debug: Intake");
    //   intakeTab.add("Target Speed", 0)
    //     .withSize(4,2)
    //     .withPosition(0,0)
    //     .withWidget("Number Slider")
    //     .withProperties(Map.of("min_value",-1.0,"max_value",1.0,"divisions",5))
    //     .getEntry();
    //   intakeTab.add("Apply Target", new InstantCommand(() -> setSpeedPercent(getNewSpeed())).ignoringDisable(true))
    //     .withSize(4, 2)
    //     .withPosition(4, 0);  
    //   intakeTab.add("Shooter Stop", new InstantCommand(() -> setSpeedPercent(0)).ignoringDisable(true))
    //     .withSize(4, 2)
    //     .withPosition(0, 2);  
    //   intakeTab.add("Shooter 100%", new InstantCommand(() -> setSpeedPercent(1)).ignoringDisable(true))
    //     .withSize(4, 2)
    //     .withPosition(4, 2);  
    // }
  }

  public double getTargetSpeed() {
    return target_speed;
  }

  /**
   * Gets the speed of the shooter
   * @return The speed of the shooter in revolutions per second
   */
  public double getCurrentSpeed() {
    return 0.0; //m_motor1.getVelocity().getValueAsDouble();
  }

  public double getSpeedPercent() {
    return getCurrentSpeed() / Constants.Shooter.kMaxRPS;
  }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    m_motor1.set(ControlMode.PercentOutput, speed);
  }

  public void stopShooter() {
    setSpeedPercent(0);
  }

  /**
   * updateDashboard is called periodically and is responsible for sending telemetry data from
   * this subsystem to the Dashboard
   */
  public void updateDashboard() {
    // Dashboard.Shooter.setShooterSpeed(getCurrentSpeed());
  }

}
