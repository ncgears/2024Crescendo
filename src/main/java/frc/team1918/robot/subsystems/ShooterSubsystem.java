
package frc.team1918.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
// import frc.team1918.robot.utils.TunableNumber;

/**
 * The Template Subsystem handles getting and managing the Template.
 * It is responsible for doing some stuff.
 */
@Deprecated(since = "2024")
public class ShooterSubsystem extends SubsystemBase {
	private static ShooterSubsystem instance;
  private static DashboardSubsystem m_dashboard = DashboardSubsystem.getInstance();
  //private and public variables defined here
  public double target_speed = 0.0;
  private double m_oldspeed = 0.0;
  private WPI_TalonFX top;
  private WPI_TalonFX bottom;

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
    top = new WPI_TalonFX(Constants.Shooter.Top.kMotorID, Constants.Shooter.canBus);
    bottom = new WPI_TalonFX(Constants.Shooter.Bottom.kMotorID, Constants.Shooter.canBus);
  }
  
  @Override
  public void periodic() {
    doStuff();
    updateDashboard();
  }

  /**
   * doStuff is called periodically and is responsible for performing the actions for
   * this subsystem. It is separate from the periodic method solely for code organization.
   */
  public void doStuff() {
    // target_speed = getTunableSpeed();
    if(getTunableSpeed() != target_speed) {
      Helpers.Debug.debug("Shooter: New Target Speed is "+getTunableSpeed());
      target_speed = getTunableSpeed();
    }
  }

  public double getTunableSpeed() {
    if(!Constants.Global.tuningMode) return target_speed;
    return m_dashboard.shooter_target.getDouble(0);
  }

  public double getCurrentSpeed() {
    return target_speed; //TODO: actually, use this to get RPMs of motor, since target_speed is public
  }

  public void setSpeedPercent(double speed) {
    //set the shooter motor speed by percent
  }

  public void stopShooter() {
    // target_speed = 0.0;
    if(Constants.Global.tuningMode) m_dashboard.shooter_target.setDouble(0.0);
  }

  /**
   * updateDashboard is called periodically and is responsible for sending telemetry data from
   * this subsystem to the Dashboard
   */
  public void updateDashboard() {
  }

}
