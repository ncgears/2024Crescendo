
package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;

/**
 * The Template Subsystem handles getting and managing the Template.
 * It is responsible for doing some stuff.
 */
public class TemplateSubsystem extends SubsystemBase {
	private static TemplateSubsystem instance;
  //private and public variables defined here

  /**
	 * Returns the instance of the TemplateSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return TemplateSubsystem instance
	 */
  public static TemplateSubsystem getInstance() {
		if (instance == null)
			instance = new TemplateSubsystem();
		return instance;
	}
  
  public TemplateSubsystem() {
    //initialize values for private and public variables, etc.
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
  }

  /**
   * updateDashboard is called periodically and is responsible for sending telemetry data from
   * this subsystem to the Dashboard
   */
  public void updateDashboard() {
  }

}
