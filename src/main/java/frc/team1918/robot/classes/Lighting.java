
package frc.team1918.robot.classes;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import frc.team1918.robot.Constants;

/**
 * The Lighting Subsystem handles getting and managing the Lighting from the CANdle.
 * It is responsible for changing lighting colors and animations.
 */
@SuppressWarnings("unused")
public class Lighting {
	private static Lighting instance;
  private final CANdle m_candle = new CANdle(Constants.Lighting.kCandleID, Constants.Lighting.canBus);
  private Colors m_currentColor, m_oldColor = Colors.OFF;
  private boolean m_blinking, m_oldBlinking = false;
  private int m_intensity, m_oldIntensity = 100;
  //private and public variables defined here
  public enum Colors {
    OFF(0,0,0),
    NCBLUE(0,119,181),
    NCGREEN(0,173,80),
    AMBER(255,100,0),
    AQUA(50,255,255),
    BLACK(0,0,0),
    BLUE(0,0,255),
    CYAN(0,255,255),
    GOLD(255,222,30),
    GREEN(0,255,0),
    JADE(0,255,40),
    MAGENTA(255,0,20),
    LACE(253,245,230),
    ORANGE(255,40,0),
    PINK(242,90,255),
    PURPLE(180,0,255),
    RED(255,0,0),
    WHITE(255,255,255),
    TEAL(0,255,120),
    YELLOW(255,150,0);
    private final int r, g, b;
    Colors(int r, int g, int b) { this.r = r; this.g = g; this.b = b; }
    public int R() { return this.r; }
    public int G() { return this.g; }
    public int B() { return this.b; }
  }
  /**
	 * Returns the instance of the LightingSubsystem subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return LightingSubsystem instance
	 */
  public static Lighting getInstance() {
		if (instance == null)
			instance = new Lighting();
		return instance;
	}
  
  public Lighting() {
    //initialize values for private and public variables, etc.
    m_currentColor = Colors.OFF;

    createDashboards();
  }
  
  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("LED Color", this::getColor)
      .withSize(5, 4)
      .withWidget("Single Color View")
      .withPosition(19, 0);  
		if(Constants.Lighting.debugDashboard) {
      ShuffleboardTab lightingTab = Shuffleboard.getTab("Debug: Lighting");
      lightingTab.addString("LED Color", this::getColor)
        .withSize(6, 4)
        .withWidget("Single Color View")
        .withPosition(0, 0);  
      lightingTab.addString("LED Color Hex", this::getColor)
        .withSize(6, 2)
        .withWidget("Text String")
        .withPosition(0, 4);  
    }
  }

  public String getColor() {
    if(m_currentColor == null) m_currentColor = Colors.OFF;
    return new Color(m_currentColor.R(), m_currentColor.G(), m_currentColor.B()).toHexString();
  }

  public void setColor(Colors color) {
    if(color != m_currentColor) { //change it
      m_currentColor = color;
      m_candle.setLEDs(color.R(), color.G(), color.B());
    }
  }

}