//OI = Operator Interface
package frc.team1918.robot.classes;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.commands.shooter.shooter_stopShooter;
import frc.team1918.robot.subsystems.ShooterSubsystem;
/**
 * This class reads and writes values to/from the SmartDashboard
 */
public class Dashboard {
	private static Dashboard instance;
    //gettable values must be configured as NetworkTableEntry
    public GenericEntry shooter_target = null;
    public GenericEntry shooter_speed = null;
    //tuning tab
    private ShuffleboardTab tuning = Shuffleboard.getTab("Tuning");
    
    public Dashboard() { //init the class
        Helpers.Debug.debug("Init dashboard subsystem");
        buildTab(tuning.getTitle());
    }

    public void periodic() {
    }

    /**
	 * Returns the instance of the Dashboard subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return DashboardSubsystem instance
	 */
    public static Dashboard getInstance() {
		if (instance == null)
			instance = new Dashboard();
		return instance;
	}

    private void buildTab(String tabTitle) {
        switch (tabTitle) {
            case "Tuning2":
                Helpers.Debug.debug("Building dashboard tab \""+tabTitle+"\"");
                shooter_target = tuning.add("Shooter Speed", 0)
                    .withSize(3,2)
                    .withPosition(0,0)
                    .withWidget("Number Slider")
                    .withProperties(Map.of("min_value",-1.0,"max_value",1.0,"divisions",5))
                    .getEntry();
                tuning.add("Stop Shooter", new shooter_stopShooter(ShooterSubsystem.getInstance()))
                    .withSize(2, 2)
                    .withPosition(3, 0);
                shooter_speed = tuning.add("Current Speed", 0)
                    .withSize(3,2)
                    .withPosition(0,2)
                    .withWidget("Number Bar")
                    .withProperties(Map.of("min_value",-1.0,"max_value",1.0,"divisions",5))
                    .getEntry();
                break;
            default:
                Helpers.Debug.debug("Unable to build dashboard tab");
                //do nothing
        }
    }
}