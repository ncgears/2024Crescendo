/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

//Global imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team1918.robot.subsystems.AimerSubsystem;
import frc.team1918.robot.subsystems.ArmSubsystem;
import frc.team1918.robot.subsystems.ClimberSubsystem;
//Util imports
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.IndexerSubsystem;
import frc.team1918.robot.subsystems.IntakeSubsystem;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import frc.team1918.robot.utils.Alert;
import frc.team1918.robot.utils.CTREConfigs;
import frc.team1918.robot.utils.TunableNumber;
import frc.team1918.robot.utils.Alert.AlertType;
//Commands imports
// import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.gyro.gyro_resetGyro;
import frc.team1918.robot.modules.SwerveModule;
import frc.team1918.robot.commands.drive.*;
import frc.team1918.robot.commands.gyro.*;
import frc.team1918.robot.classes.Gyro;
import frc.team1918.robot.classes.Lighting;
import frc.team1918.robot.classes.NCOrchestra;
import frc.team1918.robot.classes.NCPose;
import frc.team1918.robot.classes.Vision;
import frc.team1918.robot.classes.Lighting.Colors;
//CommandGroup imports
// import frc.team1918.robot.commandgroups.*;

//Phoenix6
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    public static final CTREConfigs ctreConfigs = new CTREConfigs();
    public static final Lighting lighting = Lighting.getInstance();
    public static final Gyro gyro = Gyro.getInstance();
    public static final Vision vision = Vision.getInstance();
    public static final DriveSubsystem drive = DriveSubsystem.getInstance(); //must be after gyro
    public static final NCPose pose = NCPose.getInstance(); //must be after drive
    private final NCOrchestra m_orchestra = NCOrchestra.getInstance();
    // private final PowerDistribution m_power = new PowerDistribution(0, ModuleType.kRev);
    private final PowerDistribution pdh = new PowerDistribution();
    public static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    public static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final AimerSubsystem aimer = AimerSubsystem.getInstance();
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public static final ArmSubsystem arm = ArmSubsystem.getInstance();

    private static final Alert enabledAlert = new Alert("Robot is Enabled", AlertType.INFO);

    public static Optional<Alliance> m_alliance;

    //Sendables definitions
    private SendableChooser<Command> m_auto_chooser = new SendableChooser<>();

    private static Trigger disabled() { //register a trigger for the disabled event, which is used to reset the robot
      enabledAlert.set(false);
      return new Trigger(DriverStation::isDisabled);
    }
    private static Trigger enabled() {
      enabledAlert.set(true);
      return new Trigger(DriverStation::isEnabled);
    }

    //init joysticks
    private final CommandStadiaController dj = new CommandStadiaController(Constants.OI.OI_JOY_DRIVER);
    private final CommandStadiaController oj = new CommandStadiaController(Constants.OI.OI_JOY_OPER);

   /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the bindings (buttons, triggers, etc.)
    registerCommands();
    configureBindings();
    buildDashboards();

    // Enable closed loop control of compressor and enable it
    // if(Constants.Air.isDisabled) m_air.disable();

    if(Constants.Audio.isEnabled) {
	    ArrayList<TalonFX> instrumentsAll = new ArrayList<>();
		  for (TalonFX motor: drive.getMotors()) {
			  instrumentsAll.add(motor);
		  }
		  // for (TalonFX motor: shooter.getMotors()) {
			//   instrumentsAll.add(motor);
		  // }
      // instrumentsAll.add(aimer.getMotors());
      TalonFX[] instruments = instrumentsAll.toArray(new TalonFX[instrumentsAll.size()]);
      m_orchestra.apply(instruments);
    }

    // Enable the camera server and start capture
    if(Constants.Global.CAMERA_ENABLED) {
      // UsbCamera cam = CameraServer.startAutomaticCapture();
      // cam.setResolution(320, 240);
      // cam.setFPS(25);
    }

    // Set the default command that is run for the robot. Normally, this is the drive command
    if(!Constants.DriveTrain.isDisabled) {
      drive.setDefaultCommand(
        new drive_defaultDrive(
          drive,
          () -> Helpers.OI.ncdeadband(dj.getLeftY(),false),
          () -> Helpers.OI.ncdeadband(dj.getLeftX(),false),
          () -> Helpers.OI.ncdeadband(dj.getRightX(),true)
        )
      );
    }
 
    if(!Constants.Climber.isDisabled) {
      climber.setDefaultCommand(
        climber.run(() -> climber.climberMove(Helpers.OI.ncdeadband(-oj.getRightY(),false)))
      );
    }
  }

  /**
   * This performs robot reset initializations when the disabled() trigger fires.
   */
  private void resetRobot() {
    lighting.init();
    pose.init();
    drive.init();
    aimer.init();
    climber.init();
    indexer.init();
    intake.init();
    shooter.init();
    arm.init();
  }

  // Returns true if the alliance is red, otherwise false (blue)
  public static boolean isAllianceRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
  //old version
  public static boolean isAllianceRed2() {
    m_alliance = DriverStation.getAlliance();
    if(m_alliance.isPresent()) {
      return m_alliance.get() == Alliance.Red;
    }
    return false;
  }

  private void configureBindings() {
    /** New for 2023:
    * onTrue (replaces whenPressed and whenActive): schedule on rising edge 
    * onFalse (replaces whenReleased and whenInactive): schedule on falling edge
    * whileTrue (replaces whileActiveOnce): schedule on rising edge, cancel on falling edge
    * toggleOnTrue (replaces toggleWhenActive): on rising edge, schedule if unscheduled and cancel if scheduled
    */

    /** New for 2024:
    * Since we now have a StadiaController class with a Command wrapper, we can now bind commands directly to the buttons.
    * This should make it much easier to find the appropriate functions
    */

    // bind to the disabled() trigger which happens any time the robot is disabled
    disabled().onTrue(
      new InstantCommand(this::resetRobot).ignoringDisable(true)
      .alongWith(
        new RepeatCommand(  //Lighting Disco Party!
          lighting.setColorCommand(Colors.NCBLUE)
          .andThen(new WaitCommand(0.5))
          .andThen(lighting.setColorCommand(Colors.NCGREEN))
          .andThen(new WaitCommand(0.5))
          .andThen(lighting.setColorCommand(Colors.NCBLUE))
          .andThen(new WaitCommand(0.15))
          .andThen(lighting.setColorCommand(Colors.OFF))
          .andThen(new WaitCommand(0.15))
          .andThen(lighting.setColorCommand(Colors.NCBLUE))
          .andThen(new WaitCommand(0.15))
          .andThen(lighting.setColorCommand(Colors.NCGREEN))
          .andThen(new WaitCommand(0.25))
        ).until(disabled().negate())
      )
    );
    enabled().onTrue(
      new InstantCommand(m_orchestra::stop).ignoringDisable(true)
      .andThen(lighting.setColorCommand(Colors.OFF))
    );

    /** DRIVER JOYSTICK (dj) */
    if(Constants.Audio.isEnabled) {
      /** Manage Music - Song list
        *  Brawl-Theme.chrp
        *  Megalovania.chrp
        *  Rickroll.chrp
        *  Still-Alive.chrp
        */
      dj.google().onTrue(new InstantCommand(() -> {
        if(m_orchestra.isPlaying()) {
          m_orchestra.stop();
        } else {
          m_orchestra.withMusic("Still-Alive.chrp").play();
        }
      }).ignoringDisable(true));
    }
    // Add random offset to pose estimator to test vision correction
    dj.stadia().onTrue(new InstantCommand(() -> {
      Random rand = new Random(1918);
      var trf = new Transform2d(new Translation2d(rand.nextDouble() * 10 - 2, rand.nextDouble() * 8 - 2),
        new Rotation2d(rand.nextDouble() * 2 * Math.PI));
      pose.resetPose(gyro.getYaw().getDegrees(), pose.getPose().plus(trf));
    }).ignoringDisable(true));
    // Reset Gyro
    dj.hamburger()
      .onTrue(drive.runOnce(drive::resetDistances).ignoringDisable(true)
        .andThen(gyro::zeroHeading).ignoringDisable(true)
      );
    // Defensive Lock (brake + rotate wheels 45 degrees in an X pattern)
    // dj.rightTrigger().whileTrue(new drive_defLock(drive));
    // Test the lighting system
    dj.x().onTrue(new InstantCommand(pose::setTrackingAmp).ignoringDisable(true));
    dj.y().onTrue(new InstantCommand(pose::setTrackingSpeaker).ignoringDisable(true));
    dj.leftBumper().and(arm.atIntake)
      .onTrue(intake.runOnce(intake::intakeIn).andThen(lighting.setColorCommand(Colors.YELLOW)))
      .onFalse(
        new WaitCommand(0.5)
        .andThen(intake.runOnce(intake::intakeStop))
        .andThen(new WaitCommand(0.5))
        .andThen(lighting.setColorCommand(Colors.OFF))
      );
    // Arm Amp
    dj.leftTrigger().and(pose.isTracking.negate().or(pose.isTargetSpeaker.negate()))
      .onTrue(arm.runOnce(arm::armAmp))
      .onFalse(arm.runOnce(arm::armIntake));
    // FIRE!
    dj.rightTrigger().and(shooter.isReady.or(arm.atAmp).or(arm.atTrap))
      .onTrue(indexer.runOnce(indexer::indexerUp))
      .onFalse(indexer.runOnce(indexer::indexerStop));

    /** OPERATOR JOYSTICK (oj) */
    // Pose Tracking
    oj.leftTrigger().or(dj.rightBumper())
      .onTrue(new InstantCommand(() -> pose.trackingStart()))
      .onFalse(new InstantCommand(() -> pose.trackingStop()));
    // Aimer Tracking
    pose.isTargetSpeaker.and(oj.leftTrigger().or(dj.rightBumper()))
      .onTrue(
        aimer.runOnce(aimer::aimerStartTracking)
        .andThen(shooter.runOnce(shooter::startShooter))
      )
      .onFalse(
        aimer.runOnce(aimer::aimerStopAndStow)
        .andThen(shooter.runOnce(shooter::stopShooter))
      );
    // FIRE!
    oj.rightTrigger().and(shooter.isReady.or(arm.atAmp).or(arm.atTrap))
      .onTrue(indexer.runOnce(indexer::indexerUp))
      .onFalse(indexer.runOnce(indexer::indexerStop));
    oj.leftBumper().and(arm.atIntake)
      .onTrue(intake.runOnce(intake::intakeIn).andThen(lighting.setColorCommand(Colors.YELLOW)))
      .onFalse(
        new WaitCommand(0.5)
        .andThen(intake.runOnce(intake::intakeStop))
        .andThen(new WaitCommand(0.5))
        .andThen(lighting.setColorCommand(Colors.OFF))
      );
    oj.povDown()
      .onTrue(intake.runOnce(intake::intakeOut))
      .onFalse(intake.runOnce(intake::intakeStop));

    /** AUTONOMOUS ACTIONS */
    aimer.isTracking.or(pose.isTracking).and(aimer.isReady.negate().or(pose.isReady.negate()))
      .onTrue(lighting.setColorCommand(Colors.ORANGE));
    aimer.isTracking.or(pose.isTracking).and(aimer.isReady.and(pose.isReady))
      .onTrue(lighting.setColorCommand(Colors.GREEN));
    aimer.isTracking.negate().and(pose.isTracking.negate())
      .onTrue(lighting.setColorCommand(Colors.OFF));

    /** 
     * run the indexer if it's not full. This should also be combined with a location based trigger
     * This needs some work as it is also triggering when we move the climber/arm across the beam break
     */
    intake.isRunning.and(indexer.isFull)
      .onTrue(lighting.setColorCommand(Colors.GREEN)
        .andThen(new WaitCommand(0.25))
        .andThen(lighting.setColorCommand(Colors.OFF))
        .andThen(new WaitCommand(0.15))
        .andThen(lighting.setColorCommand(Colors.GREEN))
        .andThen(new WaitCommand(0.20))
        .andThen(lighting.setColorCommand(Colors.OFF))
    );
    // indexer.isFull.onTrue(intake.runOnce(intake::intakeOut)
    //     .andThen(new WaitCommand(3))
    //     .andThen(intake.runOnce(intake::intakeStop))
    //   )
    //   .onFalse(intake.runOnce(intake::intakeIn));

  }

  /**
   * Use this to pass the named command to the main Robot class.
   * @return command
   */
  public Command getAutonomousCommand() {
    System.out.println("Robot: Selected auton routine is "+m_auto_chooser.getSelected().getName());
    return m_auto_chooser.getSelected();
  }

  /**
   * This function returns the robot commands used in Robot.java. The purpose is to make it easier to build the possible commands
   * and handle requests for commands that don't exist rather than crashing
   * @return command
   */
  public Command getRobotCommand(String name) {
    //This selects a command (or command group) to return
    switch (name) {
      default:
        return null;
    }
  }

  //From here down is all used for building the shuffleboard
  public void buildDashboards(){
    //List of Widgets: https://github.com/Gold872/elastic-dashboard/wiki/Widgets-List-&-Properties-Reference
    buildAutonChooser();
    buildDriverTab();
    buildTab("Swerve");
    buildTab("System");
    buildPowerTab();
    buildDebugTab();
    gyro.buildDashboards();
  }

    /**
   * This method registers named commands to be used in PathPlanner autos
   */
  private void registerCommands() {
    NamedCommands.registerCommand("shooterSpeed80", new InstantCommand(() -> shooter.setSpeed(80)));
    NamedCommands.registerCommand("shooterSpeed90", new InstantCommand(() -> shooter.setSpeed(90)));
    NamedCommands.registerCommand("shooterStart", shooter.runOnce(shooter::startShooter));
    NamedCommands.registerCommand("shooterStop", shooter.runOnce(shooter::stopShooter));
    NamedCommands.registerCommand("intakeIn", intake.runOnce(intake::intakeIn));
    NamedCommands.registerCommand("intakeStop", intake.runOnce(intake::intakeStop));
    NamedCommands.registerCommand("aimerTrackingStart", aimer.runOnce(aimer::aimerStartTracking));
    NamedCommands.registerCommand("aimerTrackingStop", aimer.runOnce(aimer::aimerStopAndStow));
    NamedCommands.registerCommand("poseTrackingStart", new InstantCommand(() -> pose.trackingStart()));
    NamedCommands.registerCommand("poseTrackingStop", new InstantCommand(() -> pose.trackingStop()));
    NamedCommands.registerCommand("indexerUp", indexer.runOnce(indexer::indexerUp));
    NamedCommands.registerCommand("indexerStop", indexer.runOnce(indexer::indexerStop));
  }

  public void buildAutonChooser() {
    //This builds the auton chooser, giving driver friendly names to the commands from above
    if(Constants.Auton.isDisabled) {
      m_auto_chooser.setDefaultOption("None (Auto Disabled)", Commands.none());
    } else {
      // m_auto_chooser.setDefaultOption("Do Nothing", new cg_autonDoNothing(drive));
      m_auto_chooser = AutoBuilder.buildAutoChooser();
    }
  }

  private void buildDriverTab(){
    ShuffleboardTab driverTab = buildTab("Driver");
    // Match Time - Cannot be programmatically placed, but we put it here for informative reasons
    driverTab.add("Match Time", "")
      .withPosition(0,2)
      .withSize(8,3)
      .withProperties(Map.of("time_display_mode","Minutes and Seconds","red_start_time",15,"yellow_start_time",30)) //mode: "Seconds Only" or "Minutes and Seconds"
      .withWidget("Match Time");
    // Auton Chooser
    driverTab.add("Autonomous Chooser", m_auto_chooser)
      .withPosition(0, 5)
      .withSize(8, 2)
      .withWidget("ComboBox Chooser");
    // FMS Info - Cannot be programmatically placed, but we put it here for informative reasons
    // driverTab.add("FMS Info", "")
    //   .withPosition(0,5)
    //   .withSize(8,2)
    //   .withWidget("FMSInfo");
    // Alerts
    // driverTab.add("Alerts", "")
    //   .withPosition(8,0)
    //   .withSize(11,7)
    //   .withWidget("Alerts");
    // Camera
    // driverTab.add("Camera", Robot.camera)
    //   .withPosition(8,0)
    //   .withSize(11,7)
    //   // .withProperties(Map.of("Glyph","CAMERA_RETRO","Show Glyph",true,"Show crosshair",true,"Crosshair color","#CCCCCC","Show controls",false))
    //   .withWidget("Camera Stream");
  }

  private void buildDebugTab(){
    ShuffleboardTab debugTab = buildTab("Debug");
    debugTab.add("Command Scheduler", CommandScheduler.getInstance())
      .withPosition(0,2);      
  }

  private void buildPowerTab(){
    ShuffleboardTab powerTab = buildTab("Power");
    powerTab.add("Power", pdh)
      .withPosition(0, 0);
      // .withSize(1, 1);
  }

  private ShuffleboardTab buildTab(String tabname) {
    return Shuffleboard.getTab(tabname);
  }

}
