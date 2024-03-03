package frc.team1918.robot.classes;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;

public class Gyro implements Sendable {
	private static Gyro instance;

	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
    SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Pitch"));

	/**
	 * Returns the instance of the Gyro subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return GyroSubSystem instance
	 */
    public static Gyro getInstance() {
		if (instance == null)
			instance = new Gyro();
		return instance;
	}

	/**
	 * Initializes the GyroSubsystem class, performs setup steps, etc.
	 */
    public Gyro() {
    }

	public void buildDashboards() {
		if(Constants.Gyro.debugDashboard) {
			ShuffleboardTab debugTab = Shuffleboard.getTab("DBG:Gyro");
			debugTab.add("Value", this)
				.withSize(5, 4)
				.withPosition(0, 0)  
				.withProperties(Map.of("counter_clockwise_positive",true));
			debugTab.addNumber("Pitch", this::getPitch)
				.withSize(5, 2)
				.withPosition(0, 4)
				.withWidget("Text Display");
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.setActuator(false);
		builder.addDoubleProperty("Value", () -> getYaw().getDegrees(), null);
	}

    public void zeroHeading() {
		m_gyro.reset();
		m_gyro.zeroYaw();
		// setYawOffset(0);
		Helpers.Debug.debug("Gyro: Reset Gyro");
	}

	public AHRS getGyro() {
        return m_gyro;
	}

   	/**
     * Returns the yaw/heading of the robot.
     * @return the robot's yaw as a double in degrees, from -180 to 180
     */
	public Rotation2d getYaw() {
		// return m_gyro.getYaw();
		return Rotation2d.fromDegrees(m_gyro.getYaw() * (Constants.Gyro.kGyroReversed ? -1.0 : 1.0)); //invert to CCW Positive
	}

   	/**
     * Returns the pitch of the robot.
     * @return the robot's pitch as a double in degrees
     */
	public double getPitch() {
		return m_gyro.getPitch();
	}

   	/**
     * Returns the roll of the robot.
     * @return the robot's roll as a double in degrees
     */
	public double getRoll() {
		return m_gyro.getRoll();
	}
}
