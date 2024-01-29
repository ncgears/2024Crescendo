
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
 * This subsystem handles managing the Shooter.
 * It is responsible for adjusting the speed of the shooter wheels and requesting a note from the indexer.
 */
public class ShooterSubsystem extends SubsystemBase {
	private static ShooterSubsystem instance;
  //private and public variables defined here
  public double target_speed = 0.0;
  private VelocityVoltage m_voltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
  private NeutralOut m_brake = new NeutralOut();
  private TalonFX m_motor1, m_motor2;
  private GenericEntry new_speed;
  
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
    m_motor1 = new TalonFX(Constants.Shooter.Top.kMotorID, Constants.Shooter.canBus);
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFXConfig);
      if (status1.isOK()) break;
    }
    if(!status1.isOK()) {
      Helpers.Debug.debug("Could not initialize shooter motor1, error: " + status1.toString());
    }
    m_motor2 = new TalonFX(Constants.Shooter.Bottom.kMotorID, Constants.Shooter.canBus);
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status2 = m_motor2.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFXConfig);
      if (status2.isOK()) break;
    }
    if(!status2.isOK()) {
      Helpers.Debug.debug("Could not initialize shooter motor2, error: " + status2.toString());
    }
    m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true)); //Setup motor2 inverted from motor1 as a follower

    init();
    createDashboards();
  }
  
  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    Helpers.Debug.debug("Shooter: Initialized");
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
		if(Constants.Shooter.debugDashboard) {
      ShuffleboardTab shooterTab = Shuffleboard.getTab("DBG:Shooter");
      new_speed = shooterTab.add("Target Speed", 0)
        .withSize(4,2)
        .withPosition(0,0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min_value",-1.0,"max_value",1.0,"divisions",5))
        .getEntry();
      shooterTab.add("Apply Target", new InstantCommand(() -> setSpeedPercent(getNewSpeed())).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(4, 0);  
      shooterTab.add("Shooter Stop", new InstantCommand(() -> setSpeedPercent(0)).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(0, 2);  
      shooterTab.add("Shooter 100%", new InstantCommand(() -> setSpeedPercent(1)).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(4, 2);  
    }
  }

  public double getNewSpeed() {
    return new_speed.getDouble(0.0);
  }

  public double getTargetSpeed() {
    return target_speed;
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

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    target_speed = speed;
    //set the shooter motor speed by percent (-1 to 1)
    if(speed == 0.0) {
      Helpers.Debug.debug("Shooter Target RPS: 0.0");
      m_motor1.setControl(m_brake);
    } else {
      double rps = speed * Constants.Shooter.kMaxRPS;
      Helpers.Debug.debug("Shooter Target RPS: " + Helpers.General.roundDouble(rps, 2));
      m_motor1.setControl(m_voltageVelocity.withVelocity(rps));
    }
  }

  public void stopShooter() {
    // target_speed = 0.0;
    // if(Constants.Global.tuningMode) RobotContainer.dashboard.shooter_target.setDouble(0.0);
    Helpers.Debug.debug("Shooter: Stop");
  }
}
