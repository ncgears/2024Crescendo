package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team1918.robot.Constants;
import frc.team1918.robot.modules.Aimer;
import frc.team1918.robot.modules.ConvectionFan;
import frc.team1918.robot.modules.Burner;
import frc.team1918.robot.modules.Burner.BurnerPositions;
import frc.team1918.robot.modules.Griddle;
import frc.team1918.robot.modules.Griddle.GriddleDirections;
import frc.team1918.robot.modules.GreaseTrap;
import frc.team1918.robot.modules.GreaseTrap.GreaseTrapPositions;
import frc.team1918.robot.modules.HotPlate;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;

/**
 * The STOVE Subsystem manages the GRIDDLE (primary conveyor), HOTPLATE (scoring ramp), and GREASETRAP (flip ramp). 
 * It is responsible for delivering PANCAKES (cones) and WAFFLES (cubes) off the end if the scoring ramp or back to the flip ramp.
 * It manages the angle of the 
*/
public class StoveSubsystem extends SubsystemBase {
	private static StoveSubsystem instance;
	
	//initialize modules used in subsystem
	private Griddle m_Griddle; //conveyor module
	private GreaseTrap m_GreaseTrap; //flip ramp module
	private HotPlate m_HotPlate; //scoring ramp controller
	private Burner m_Burner; //elevator controller
	private Aimer m_Aimer; //aimer controller
	private ConvectionFan m_ConvectionFan; //thumper spike
	
	/**
	 * Returns the instance of the subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return StoveSubsystem instance
	 */
	public static StoveSubsystem getInstance() {
		if (instance == null)
			instance = new StoveSubsystem();
		return instance;
	}

	/**
	 * Initializes the IntakeSubsystem class, performs setup steps, etc.
	 */
	public StoveSubsystem() {
		m_GreaseTrap = new GreaseTrap("GreaseTrap", Constants.Stove.GreaseTrap.constants, Constants.Stove.GreaseTrap.gains);
		m_HotPlate = new HotPlate("HotPlate", Constants.Stove.HotPlate.constants, Constants.Stove.HotPlate.gains);
		m_Burner = new Burner("Burner", Constants.Stove.Burner.constants, Constants.Stove.Burner.gains);
		m_Griddle = new Griddle("Griddle", Constants.Stove.Griddle.constants, Constants.Stove.Griddle.gains);
		m_Aimer = new Aimer("Aimer", Constants.Stove.Aimer.constants, Constants.Stove.Aimer.gains);
		m_ConvectionFan = new ConvectionFan();

		//make sure we reset things on init
		m_GreaseTrap.moveTo(GreaseTrapPositions.HOME);
		m_HotPlate.moveTo(HotPlatePositions.HOME);
		m_Burner.moveTo(BurnerPositions.HOME);
		m_Griddle.setDirectionAndSpeed(GriddleDirections.STOP, 0.0);
		m_Aimer.moveTo(0.0);
		m_ConvectionFan.setConvectionFan(false);
	}

	/**
	 * Handles things that should be done every iteration, such as updating the dashboard or checking the status of things.
	 */
	@Override
	public void periodic() {
		updateDashboard();
	}

	public void updateDashboard() {
		m_HotPlate.updateDashboard();
		m_Burner.updateDashboard();
		m_Griddle.updateDashboard();
		m_GreaseTrap.updateDashboard();
		m_Aimer.updateDashboard();
		m_ConvectionFan.updateDashboard();
	}

	/**
	 * Sets the Griddle direction (GriddleDirections) and speed (double)
	 * @param direction GriddleDirections enum representing the direction of griddle
	 * @param speed Double representing the percentage for the griddle
	 */
	public void setGriddleDirectionAndSpeed(GriddleDirections direction, double speed) {
		m_Griddle.setDirectionAndSpeed(direction, speed);
	}

	/**
	 * Sets the GreaseTrap speed manually (for zeroing). 
	 * This will only set a negative speed, regardless whether positive or negative supplied
	 * @param speed
	 */
	public void setGreaseTrapZeroSpeed(double speed) {
		m_GreaseTrap.currentPosition = GreaseTrapPositions.ZERO;
		m_GreaseTrap.setSpeed(Math.abs(speed) * -1.0);
	}

	/**
	 * Returns whether the module is at its reverse limit
	 * @return (boolean) limit switch state for reverse limit
	 */
	public boolean getGreaseTrapRevLimit() {
		return m_GreaseTrap.isRevLimit();
	}

	/**
	 * Returns the current GreaseTrap Position
	 * @return (GreaseTrapPositions) position of greasetrap
	 */
	public GreaseTrapPositions getGreaseTrapPosition() {
		return m_GreaseTrap.currentPosition;
	}

	/**
	 * Sets the GreaseTrap current position to zero
	 */
	public void setGreaseTrapZeroPos() {
		m_GreaseTrap.setZeroPos();
	}

	/**
	 * Moves the GreaseTrap to a designated position
	 * @param position - This is a GreaseTrapPositions enum of {@GreaseTrapPositions}
	 */
	public void moveGreaseTrapTo(GreaseTrapPositions position) {
		m_GreaseTrap.moveTo(position);
	}

	/**
	 * Sets the HotPlate speed manually (for zeroing). 
	 * This will only set a negative speed, regardless whether positive or negative supplied
	 * @param speed
	 */
	public void setHotPlateZeroSpeed(double speed) {
		m_HotPlate.currentPosition = HotPlatePositions.ZERO;
		m_HotPlate.setSpeed(Math.abs(speed) * -1.0);
	}

	/**
	 * Returns whether the module is at its reverse limit
	 * @return (boolean) limit switch state for reverse limit
	 */
	public boolean getHotPlateRevLimit() {
		return m_HotPlate.isRevLimit();
	}

	/**
	 * Moves the HotPlate to a designated position
	 * @param position - This is a HotPlatePositions enum of {@HotPlatePositions}
	 */
	public void moveHotPlateTo(HotPlatePositions position) {
		m_HotPlate.moveTo(position);
	}

	/**
	 * Sets the HotPlate current position to zero
	 */
	public void setHotPlateZeroPos() {
		m_HotPlate.setZeroPos();
	}

	/**
	 * Returns the current HotPlate Position
	 * @return (HotPlatePositions) position of greasetrap
	 */
	public HotPlatePositions getHotPlatePosition() {
		return m_HotPlate.currentPosition;
	}

	/**
	 * Sets the Burner speed manually (for zeroing). 
	 * This will only set a negative speed, regardless whether positive or negative supplied
	 * @param speed
	 */
	public void setBurnerZeroSpeed(double speed) {
		m_Burner.currentPosition = BurnerPositions.ZERO;
		m_Burner.setSpeed(Math.abs(speed) * -1.0);
	}

	/**
	 * Returns whether the module is at its reverse limit
	 * @return (boolean) limit switch state for reverse limit
	 */
	public boolean getBurnerRevLimit() {
		return m_Burner.isRevLimit();
	}

	/**
	 * Returns whether the module is at its forward limit
	 * @return (boolean) limit switch state for reverse limit
	 */
	public boolean getBurnerFwdLimit() {
		return m_Burner.isFwdLimit();
	}

	/**
	 * Returns the current Burner Position
	 * @return (BurnerPositions) position of greasetrap
	 */
	public BurnerPositions getBurnerPosition() {
		return m_Burner.currentPosition;
	}

	/**
	 * Sets the Burner current position to zero
	 */
	public void setBurnerZeroPos() {
		m_Burner.setZeroPos();
	}

	/**
	 * Moves the Burner to a designated position
	 * @param position - This is a BurnerPositions enum of {@BurnerPositions}
	 */
	public void moveBurnerTo(BurnerPositions position) {
		m_Burner.moveTo(position);
	}

	public void moveAimer(double speed) {
		m_Aimer.setSpeed(speed);
	}

	public void moveAimerTo(double position) {
		m_Aimer.moveTo(0.0);
	}

	public void setConvectionFan(boolean enabled) {
		m_ConvectionFan.setConvectionFan(enabled);
	}

}