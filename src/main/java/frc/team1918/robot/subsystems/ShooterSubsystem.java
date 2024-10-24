
package frc.team1918.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team1918.robot.constants.*; 
import frc.team1918.robot.Helpers;
import frc.team1918.robot.Robot;
// import frc.team1918.robot.utils.TunableNumber;
import frc.team1918.robot.RobotContainer;

/**
 * This subsystem handles managing the Shooter.
 * It is responsible for adjusting the speed of the shooter wheels and requesting a note from the indexer.
 */
public class ShooterSubsystem extends SubsystemBase {
	private static ShooterSubsystem instance;
  //private and public variables defined here
  public double target_speed = 90.0;
  private final MotionMagicVelocityVoltage m_mmVelocityVoltage = new MotionMagicVelocityVoltage(0);
  // private final VelocityVoltage m_mmVelocityVoltage = new VelocityVoltage(0);
  private NeutralOut m_brake = new NeutralOut();
  private TalonFX m_motor1, m_motor2;
  private DoubleSubscriber new_speed_sub;
  private double new_speed = 90.0;
  private enum State {
    READY(DashboardConstants.Colors.GREEN),
    START(DashboardConstants.Colors.ORANGE),
    ERROR(DashboardConstants.Colors.RED),
    STOP(DashboardConstants.Colors.BLACK);
    private final String color;
    State(String color) { this.color = color; }
    public String getColor() { return this.color; }
  }
  private State m_curState = State.STOP;
  public final Trigger isReady = new Trigger(this::atSetpoint);
  public final Trigger isAcceptable = new Trigger(this::atShootable);

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

  private final VoltageOut m_sysidControl = new VoltageOut(0);
  private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
  private SysIdRoutine m_SysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Default ramp rate is acceptable
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
            null,          // Default timeout is acceptable
                                    // Log state with Phoenix SignalLogger class
            (state)->SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts)-> m_motor1.setControl(m_sysidControl.withOutput(volts.in(Volts))),
            null,
            this));
  public Command joystickDriveCommand(DoubleSupplier output) {
      return run(()->m_motor1.setControl(m_joystickControl.withOutput(output.getAsDouble())));
  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_SysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_SysIdRoutine.dynamic(direction);
  }

  public ShooterSubsystem() {
    setName("Shooter");
    //initialize values for private and public variables, etc.
    m_motor1 = new TalonFX(ShooterConstants.Top.kMotorID, ShooterConstants.canBus);
    StatusCode status1 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status1 = m_motor1.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFXConfig);
      if (status1.isOK()) break;
    }
    if(!status1.isOK()) {
      Helpers.Debug.debug("Could not initialize shooter motor1, error: " + status1.toString());
    }
    m_motor2 = new TalonFX(ShooterConstants.Bottom.kMotorID, ShooterConstants.canBus);
    StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status2 = m_motor2.getConfigurator().apply(RobotContainer.ctreConfigs.shooterFXConfig);
      if (status2.isOK()) break;
    }
    if(!status2.isOK()) {
      Helpers.Debug.debug("Could not initialize shooter motor2, error: " + status2.toString());
    }
    m_motor2.setInverted(ShooterConstants.Bottom.kIsInverted);
    // Dont use a follower for disconnected mechanical systems
    // m_motor2.setControl(new Follower(m_motor1.getDeviceID(), true)); //Setup motor2 inverted from motor1 as a follower

    /** All this is for cleaning CANbus utilization
    // This changes the frequency of updates from signals we are interested in 
    BaseStatusSignal.setUpdateFrequencyForAll(250,
      m_motor1.getPosition(),
      m_motor1.getVelocity(),
      m_motor1.getMotorVoltage(),
      m_motor2.getClosedLoopReference()
      );
    BaseStatusSignal.setUpdateFrequencyForAll(250,
      m_motor2.getPosition(),
      m_motor2.getVelocity(),
      m_motor2.getMotorVoltage(),
      m_motor2.getClosedLoopReference()
      );
    
    // Optimize out the other signals, since they're not particularly helpful for us 
    m_motor1.optimizeBusUtilization();
    m_motor2.optimizeBusUtilization();
    */

    /** */
    //Sets the save location for signal logger and starts it, needed for sysid
    // SignalLogger.setPath("/home/lvuser/logs/");
    // SignalLogger.start();
    /** */

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
    new_speed = new_speed_sub.get(0.0);
    updateState();
  }

  public void createDashboards() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addString("Shooter", this::getColor)
      .withSize(2, 2)
      .withPosition(14, 7)
      .withWidget("Single Color View");

    if(ShooterConstants.debugDashboard) {
      ShuffleboardTab systemTab = Shuffleboard.getTab("System");
      ShuffleboardLayout shooterList = systemTab.getLayout("Shooter", BuiltInLayouts.kList)
				.withSize(4,5)
				.withPosition(12,0)
				.withProperties(Map.of("Label position","LEFT"));
			shooterList.addString("Status", this::getColor)
        .withWidget("Single Color View");
      shooterList.addString("State", this::getStateName);
      shooterList.addNumber("Target Speed (RPS)", this::getTargetSpeed);
			shooterList.addNumber("Speed (Top)", this::getCurrentTopSpeed);
			shooterList.addNumber("Speed (Bottom)", this::getCurrentBottomSpeed);
      
      ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
      ShuffleboardLayout dbgShooterList = debugTab.getLayout("Shooter", BuiltInLayouts.kList)
				.withSize(4,9)
				.withPosition(4,0)
				.withProperties(Map.of("Label position","LEFT"));
			dbgShooterList.addString("Status", this::getColor)
        .withWidget("Single Color View");
			dbgShooterList.addNumber("Target Speed (RPS)", this::getTargetSpeed);
			dbgShooterList.addNumber("Current Speed (RPS)", this::getCurrentTopSpeed);
      dbgShooterList.add("Stop", new InstantCommand(() -> setSpeed(0)))
        .withProperties(Map.of("show_type",false));  
      dbgShooterList.add("60 RPS", new InstantCommand(() -> setSpeed(60)))
        .withProperties(Map.of("show_type",false));  
      dbgShooterList.add("85 RPS", new InstantCommand(() -> setSpeed(85)))
        .withProperties(Map.of("show_type",false));  
      dbgShooterList.add("90 RPS", new InstantCommand(() -> setSpeed(90)))
        .withProperties(Map.of("show_type",false));  
      dbgShooterList.add("95 RPS", new InstantCommand(() -> setSpeed(95)))
        .withProperties(Map.of("show_type",false)); 

      debugTab.addNumber("Shooter Target (RPS)", this::getNewSpeed)
        .withSize(4,2)
        .withPosition(8,0)
        .withWidget("Number Slider")
        .withProperties(Map.of("min_value",-ShooterConstants.kMaxRPS,"max_value",ShooterConstants.kMaxRPS,"divisions",10));
      debugTab.add("Apply Shooter Speed", new InstantCommand(() -> setTarget(getNewSpeed())).ignoringDisable(true))
        .withSize(4, 2)
        .withPosition(8, 2)
        .withProperties(Map.of("show_type",false));  

      new_speed_sub = NetworkTableInstance.getDefault().getDoubleTopic("/Shuffleboard/Debug/Shooter Target (RPS)").subscribe(0.0);
    }
  }

  public State getState() { return m_curState; }
  public String getStateName() { return m_curState.toString(); }
  public double getNewSpeedPercent() { return Helpers.General.roundDouble(new_speed / ShooterConstants.kMaxRPS,2); }
  public double getNewSpeed() { return Helpers.General.roundDouble(new_speed,2); }
  public double getTargetSpeed() { return target_speed; }
  public String getColor() { return m_curState.getColor(); }

  private void updateState() {
    if(m_curState == State.START || m_curState == State.READY)
      m_curState = (atSetpoint()) ? State.READY : State.START;
  }

  private boolean atSetpoint() {
    if(Robot.isSimulation()) return true;
    return (
      MathUtil.isNear(
        m_motor1.getClosedLoopReference().getValue(), 
        m_motor1.getVelocity().getValue(), 
        ShooterConstants.kSpeedToleranceOptimal
      )
      && MathUtil.isNear(
        m_motor2.getClosedLoopReference().getValue(),
        m_motor2.getVelocity().getValue(),
        ShooterConstants.kSpeedToleranceOptimal
      )
    );
    // double error1 = m_motor1.getClosedLoopReference().getValue() - m_motor1.getVelocity().getValue();
    // double error2 = m_motor2.getClosedLoopReference().getValue() - m_motor2.getVelocity().getValue();
    // return (Math.abs(error1) <= ShooterConstants.kSpeedToleranceOptimal) && (Math.abs(error2) <= ShooterConstants.kSpeedToleranceOptimal);
  }

  private boolean atShootable() {
    if(Robot.isSimulation()) return true;
    return (
      MathUtil.isNear(
        m_motor1.getClosedLoopReference().getValue(), 
        m_motor1.getVelocity().getValue(), 
        ShooterConstants.kSpeedToleranceAcceptable
      )
      && MathUtil.isNear(
        m_motor2.getClosedLoopReference().getValue(),
        m_motor2.getVelocity().getValue(),
        ShooterConstants.kSpeedToleranceAcceptable
      )
    );
    // double error1 = m_motor1.getClosedLoopReference().getValue() - m_motor1.getVelocity().getValue();
    // double error2 = m_motor2.getClosedLoopReference().getValue() - m_motor2.getVelocity().getValue();
    // return (Math.abs(error1) <= ShooterConstants.kSpeedToleranceAcceptable) && (Math.abs(error2) <= ShooterConstants.kSpeedToleranceAcceptable);
  }

  /**
   * Gets the speed of the shooter
   * @return The speed of the shooter in revolutions per second
   */
  public double getCurrentTopSpeed() { return Helpers.General.roundDouble(m_motor1.getVelocity().getValue(),2); }
  public double getCurrentBottomSpeed() { return Helpers.General.roundDouble(m_motor2.getVelocity().getValue(),2); }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in percentage (-1.0 to 1.0)
   */
  public void setSpeedPercent(double speed) {
    double rps = speed * ShooterConstants.kMaxRPS;
    setSpeed(rps);
  }

  /**
   * Sets the speed of the shooter
   * @param speed The speed of the shooter in rotations per second
   */
  public void setSpeed(double speed) {
    // speed = Math.min(Shooter.kMaxRPS,Math.max(-Shooter.kMaxRPS,speed));
    speed = MathUtil.clamp(speed, -ShooterConstants.kMaxRPS, ShooterConstants.kMaxRPS);
    target_speed = speed;
    if(speed == 0.0) {
      m_curState = State.STOP;
      Helpers.Debug.debug("Shooter Target RPS: 0.0");
      m_motor1.setControl(m_brake);
      m_motor2.setControl(m_brake);
    } else {
      m_curState = State.START;
      Helpers.Debug.debug("Shooter Target RPS: " + Helpers.General.roundDouble(speed, 2));
      m_motor1.setControl(m_mmVelocityVoltage.withVelocity(speed));
      m_motor2.setControl(m_mmVelocityVoltage.withVelocity(speed));
    }
  }

  public void stopShooter() {
    // target_speed = 0.0;
    // if(Constants.Global.tuningMode) RobotContainer.dashboard.shooter_target.setDouble(0.0);
    m_curState = State.STOP;
    m_motor1.setControl(m_brake);
    m_motor2.setControl(m_brake);
    Helpers.Debug.debug("Shooter: Stop");
  }

  public void startShooter() {
    m_curState = State.START;
    setSpeed(target_speed);
    Helpers.Debug.debug("Shooter: Start");
  }

  public void setTarget(double speed) {
    target_speed = speed;
  }

  public void updateTarget(double speed) {
    setTarget(speed);
    startShooter();
  }

  public TalonFX[] getMotors() {
    TalonFX[] motors = {m_motor1, m_motor2};
    return motors;
  }

}
