package frc.team1918.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team1918.robot.Constants;
import frc.team1918.robot.modules.Spatula;
import frc.team1918.robot.modules.Spatula.SpatulaPositions;

/**
 * The FiveSecondRule (Intake) Subsystem manages the left and right Spatulas. 
 * It is responsible for getting PANCAKES (cones) and WAFFLES (cubes) from the floor before THE FAMILY DOG (other robots)
 * and delivering them to the griddle (primary conveyor)
  */
public class FiveSecondRuleSubsystem extends SubsystemBase {
	private static FiveSecondRuleSubsystem instance;
	public enum spatulas {BOTH, LEFT, RIGHT};

	//initialize spatulas
	private static Spatula m_SpatulaLeft = new Spatula("SpatulaLeft", Constants.Spatula.Left.constants, Constants.Spatula.Left.gains, Constants.Spatula.Left.positions); // Left
	private static Spatula m_SpatulaRight = new Spatula("SpatulaRight", Constants.Spatula.Right.constants, Constants.Spatula.Right.gains, Constants.Spatula.Right.positions); // Right
	private Spatula[] modules = {m_SpatulaLeft, m_SpatulaRight};

	/**
	 * Returns the instance of the Intake subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return IntakeSubSystem instance
	 */
	public static FiveSecondRuleSubsystem getInstance() {
		if (instance == null)
			instance = new FiveSecondRuleSubsystem();
		return instance;
	}

	/**
	 * Initializes the IntakeSubsystem class, performs setup steps, etc.
	 */
	public FiveSecondRuleSubsystem() {
		// moveSpatulaTo(spatulas.BOTH, SpatulaPositions.HOME); //Make sure all spatulas are initialized to their stowed position
		moveSpatulaTo(spatulas.LEFT, SpatulaPositions.HOME);
		moveSpatulaTo(spatulas.RIGHT, SpatulaPositions.HOME);
	}

	/**
	 * Handles things that should be done every iteration, such as updating the dashboard or checking the status of things.
	 */
	@Override
	public void periodic() {
		updateDashboard();
	}

	public void updateDashboard() {
		for (Spatula module: modules) { //For each spatula, update the dashboard
			module.updateDashboard();
		}
	}

	/**
	 * Sets the Burner speed manually (for zeroing). 
	 * This will only set a negative speed, regardless whether positive or negative supplied
	 * @param speed
	 * @param spatula
	 */
	public void setSpatulaZeroSpeed(double speed, spatulas spatula) {
		speed = Math.abs(speed) * -1.0;
		switch (spatula) {
			case LEFT:
				m_SpatulaLeft.currentPosition = SpatulaPositions.ZERO;
				m_SpatulaLeft.setSpeed(speed);
				break;
			case RIGHT:
				m_SpatulaRight.currentPosition = SpatulaPositions.ZERO;
				m_SpatulaRight.setSpeed(speed);
				break;
			case BOTH:
				m_SpatulaLeft.currentPosition = SpatulaPositions.ZERO;
				m_SpatulaRight.currentPosition = SpatulaPositions.ZERO;
				m_SpatulaLeft.setSpeed(speed);
				m_SpatulaRight.setSpeed(speed);
		}
	}

	/**
	 * Sets the Burner position to zero
	 * @param spatula
	 */
	public void setSpatulaZeroPos(spatulas spatula) {
		switch (spatula) {
			case LEFT:
				m_SpatulaLeft.setZeroPos();
				break;
			case RIGHT:
				m_SpatulaRight.setZeroPos();
				break;
			default:
		}
	}

	/**
	 * Moves the Spatula to a designated position
	 * @param position - This is a SpatulaPositions enum of {@SpatulaPositions}
	 */
	public void moveSpatulaTo(spatulas spatula, SpatulaPositions position) {
		switch (spatula) {
			case LEFT:
				m_SpatulaLeft.moveTo(position);
				break;
			case RIGHT:
				m_SpatulaRight.moveTo(position);
				break;
			case BOTH:
				m_SpatulaLeft.moveTo(position);
				m_SpatulaRight.moveTo(position);
		}
	}

	public SpatulaPositions getSpatulaPosition(spatulas spatula) {
		switch (spatula) {
			case LEFT:
				return m_SpatulaLeft.currentPosition;
			case RIGHT:
				return m_SpatulaRight.currentPosition;
			default:
				return null;
		}
	}

	public boolean getSpatulaRevLimit(spatulas spatula) {
		switch (spatula) {
			case LEFT:
				return m_SpatulaLeft.isRevLimit();
			case RIGHT:
				return m_SpatulaRight.isRevLimit();
			default:
				return false;
		}
	}
}