
package frc.team1918.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  public double target_speed = 95.0;
  private VelocityVoltage m_voltageVelocity = new VelocityVoltage(0,0,true,0,0,false,false,false);
  private NeutralOut m_brake = new NeutralOut();
  private TalonFX m_motor1, m_motor2;
  private DoubleSubscriber new_speed_sub;
  private double new_speed = 95.0;
  
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
    m_motor2.setInverted(Constants.Shooter.Bottom.kIsInverted);
    // Dont use a follower for disconnected mechanical systems
    // m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true)); //Setup motor2 inverted from motor1 as a follower

    init();
    createDashboards();
  }

  public final Trigger isReady = new Trigger(this::isAtSpeed);

  /**
   * The init function resets and operational state of the subsystem
   */
  public void init() {
    Helpers.Debug.debug("Shooter: Initialized");
  }

  @Override
  public void periodic() {
    new_speed = new_speed_sub.get(0.0);
  }

  public void createDashboards() {
		if(Constants.Shooter.debugDashboard) {
      ShuffleboardTab shooterTab = Shuffleboard.getTab("DBG:Shooter");
      //This widget has a bug that Nadav is working on where it doesn't set value into NT when dropping the handle until next time
      shooterTab.addNumber("New Target (RPS)", this::getNewSpeed)
        .withSize(4,2)
        .withPosition(0,0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min_value",-Constants.Shooter.kMaxRPS,"max_value",Constants.Shooter.kMaxRPS,"divisions",10));
      shooterTab.add("Apply Target", new InstantCommand(() -> setTarget(getNewSpeed())).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(4, 0)
        .withProperties(Map.of("show_type",false));  
      shooterTab.addNumber("Current Target (RPS)", this::getTargetSpeed)
        .withSize(4,2)
        .withPosition(8,0)
        .withWidget("Text Display");
      shooterTab.addNumber("Current Speed (RPS)", this::getCurrentSpeed)
        .withSize(2,2)
        .withPosition(0,2)
        .withWidget("Text Display");
      shooterTab.add("Shooter Stop", new InstantCommand(() -> setSpeedPercent(0)))
        .withSize(4, 2)
        .withPosition(2, 2)
        .withProperties(Map.of("show_type",false));  
      shooterTab.add("Shooter 100%", new InstantCommand(() -> setSpeedPercent(1)))
        .withSize(4, 2)
        .withPosition(6, 2)
        .withProperties(Map.of("show_type",false));  
      
      new_speed_sub = NetworkTableInstance.getDefault().getDoubleTopic("/Shuffleboard/DBG:Shooter/New Target (RPS)").subscribe(0.0);
    }
  }

  public double getNewSpeedPercent() { return Helpers.General.roundDouble(new_speed / Constants.Shooter.kMaxRPS,2); }
  public double getNewSpeed() { return Helpers.General.roundDouble(new_speed,2); }
  public double getTargetSpeed() { return target_speed; }
  public boolean isAtSpeed() { 
    if(target_speed == 0.0) return false;
    double error = target_speed - m_motor1.getVelocity().getValue();
    return (Math.abs(error) <= Constants.Shooter.kSpeedTolerance);
  }

  /**
   * Gets the speed of the shooter
   * @return The speed of the shooter in revolutions per second
   */
  public double getCurrentSpeed() { return Helpers.General.roundDouble(m_motor1.getVelocity().getValue(),2); }
  public double getCurrentSpeedPercent() { return getCurrentSpeed() / Constants.Shooter.kMaxRPS; }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    double rps = speed * Constants.Shooter.kMaxRPS;
    setSpeed(rps);
  }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in rotations per second
   */
  public void setSpeed(double speed) {
    // speed = Math.min(Constants.Shooter.kMaxRPS,Math.max(-Constants.Shooter.kMaxRPS,speed));
    speed = MathUtil.clamp(speed, -Constants.Shooter.kMaxRPS, Constants.Shooter.kMaxRPS);
    target_speed = speed;
    if(speed == 0.0) {
      Helpers.Debug.debug("Shooter Target RPS: 0.0");
      m_motor1.setControl(m_brake);
      m_motor2.setControl(m_brake);
    } else {
      Helpers.Debug.debug("Shooter Target RPS: " + Helpers.General.roundDouble(speed, 2));
      m_motor1.setControl(m_voltageVelocity.withVelocity(speed));
      m_motor2.setControl(m_voltageVelocity.withVelocity(speed));
    }
  }

  public void stopShooter() {
    // target_speed = 0.0;
    // if(Constants.Global.tuningMode) RobotContainer.dashboard.shooter_target.setDouble(0.0);
    m_motor1.setControl(m_brake);
    m_motor2.setControl(m_brake);
    Helpers.Debug.debug("Shooter: Stop");
  }

  public void startShooter() {
    setSpeed(target_speed);
    Helpers.Debug.debug("Shooter: Start");
  }

  private void setTarget(double speed) {
    target_speed = speed;
  }

  public TalonFX[] getMotors() {
    TalonFX[] motors = {m_motor1, m_motor2};
    return motors;
  }

}
