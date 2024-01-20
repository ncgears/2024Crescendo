
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
// import frc.team1918.robot.utils.TunableNumber;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.commands.shooter.shooter_stopShooter;
import frc.team1918.robot.subsystems.LightingSubsystem.Colors;

/**
 * The Template Subsystem handles getting and managing the Template.
 * It is responsible for doing some stuff.
 */
public class ShooterSubsystem extends SubsystemBase {
	private static ShooterSubsystem instance;
  //private and public variables defined here
  public double target_speed = 0.0;
  private double m_oldspeed = 0.0;
  private VelocityVoltage m_voltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
  private NeutralOut m_brake = new NeutralOut();
  private TalonFX m_motor1;
  private final ShuffleboardTab shooter = Shuffleboard.getTab("Shooter");
  
  /**
	 * Returns the instance of the ShooterSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return ShooterSubsystem instance
	 */
  public static ShooterSubsystem getInstance() {
		if (instance == null)
			instance = new ShooterSubsystem();
		return instance;
	}
  
  public ShooterSubsystem() {
    //initialize values for private and public variables, etc.
    m_motor1 = new TalonFX(Constants.Shooter.kMotorID, Constants.Shooter.canBus);
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFXConfig);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      Helpers.Debug.debug("Could not initialize shooter controller, error: " + status.toString());
    }

		//Add this sendable to the Dashboard
		SmartDashboard.putData("Shooter", this);
    shooter.add("Target Speed", this)
      .withSize(3,2)
      .withPosition(0,0)
      .withWidget("Number Slider")
      .withProperties(Map.of("min_value",-1.0,"max_value",1.0,"divisions",5));
    shooter.add("Shooter 100%", new InstantCommand(() -> setSpeedPercent(1)).ignoringDisable(true))
      .withSize(2, 2)
      .withPosition(3, 0);  
  }
  
  @Override
  public void periodic() {
    doStuff();
    updateDashboard();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("Number Slider");
    builder.setActuator(true);
    builder.addDoubleProperty("Target Speed", this::getTargetSpeed, this::setSpeedPercent);
    builder.addDoubleProperty("Current Speed", this::getSpeedPercent, null);
  }

  /**
   * doStuff is called periodically and is responsible for performing the actions for
   * this subsystem. It is separate from the periodic method solely for code organization.
   */
  public void doStuff() {
    // target_speed = getTunableSpeed();
    if(getTunableSpeed() != target_speed) {
      // Helpers.Debug.debug("Shooter: New Target Speed is "+getTunableSpeed());
      target_speed = getTunableSpeed();
      setSpeedPercent(target_speed);
    }
  }

  public double getTunableSpeed() {
    if(!Constants.Global.tuningMode) return target_speed;
    return 0.0;
    // return RobotContainer.dashboard.shooter_target.getDouble(0);
  }

  /**
   * Gets the speed of the shooter
   * @return The speed of the shooter in revolutions per second
   */
  public double getCurrentSpeed() {
    return m_motor1.getVelocity().getValueAsDouble();
  }

  public double getSpeedPercent() {
    return getCurrentSpeed() / Constants.Shooter.kMaxRPS;
  }

  public double getTargetSpeed() {
    return target_speed;
  }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    //set the shooter motor speed by percent (-1 to 1)
    if(speed == 0.0) {
      // Helpers.Debug.debug("Stopping shooter");
      m_motor1.setControl(m_brake);
    } else {
      double rps = speed * Constants.Shooter.kMaxRPS;
      Helpers.Debug.debug("New requested velocity RPS is " + rps);
      m_motor1.setControl(m_voltageVelocity.withVelocity(rps));
    }
  }

  public void stopShooter() {
    // target_speed = 0.0;
    // if(Constants.Global.tuningMode) RobotContainer.dashboard.shooter_target.setDouble(0.0);
  }

  /**
   * updateDashboard is called periodically and is responsible for sending telemetry data from
   * this subsystem to the Dashboard
   */
  public void updateDashboard() {
    // Dashboard.Shooter.setShooterSpeed(getCurrentSpeed());
  }

}
