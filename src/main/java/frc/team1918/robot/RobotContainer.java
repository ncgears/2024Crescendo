/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

//Global imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Map;
import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team1918.robot.subsystems.AimerSubsystem;
import frc.team1918.robot.subsystems.ClimberSubsystem;
//Util imports
import frc.team1918.robot.subsystems.CommandSwerveDrivetrain;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.IndexerSubsystem;
import frc.team1918.robot.subsystems.IntakeSubsystem;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import frc.team1918.robot.utils.CTREConfigs;
import frc.team1918.robot.utils.TunableNumber;
//Commands imports
// import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.gyro.gyro_resetGyro;
import frc.team1918.robot.commands.drive.*;
import frc.team1918.robot.commands.gyro.*;
import frc.team1918.robot.generated.TunerConstants;
import frc.team1918.robot.classes.Gyro;
import frc.team1918.robot.classes.Lighting;
import frc.team1918.robot.classes.Vision;
import frc.team1918.robot.classes.Lighting.Colors;
//CommandGroup imports
import frc.team1918.robot.commandgroups.*;

//Phoenix6
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

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

  //subsystems definitions
    // private final PowerDistribution m_pdp = new PowerDistribution();
    // private final Compressor m_air = new Compressor(PneumaticsModuleType.CTREPCM);
    private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
    private final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
    private final IndexerSubsystem m_indexer = IndexerSubsystem.getInstance();
    private final ClimberSubsystem m_climber = ClimberSubsystem.getInstance();
    private final AimerSubsystem m_aimer = AimerSubsystem.getInstance();
    private final ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();

  //Sendables definitions
    private SendableChooser<Command> m_auto_chooser = new SendableChooser<>();
    private static SendableChooser<String> m_auto_cone = new SendableChooser<>();
    private static SendableChooser<String> m_auto_burner = new SendableChooser<>();

    private static Trigger disabled() { //register a trigger for the disabled event, which is used to reset the robot
      return new Trigger(DriverStation::isDisabled);
    }
   
    //CommandSwerve stuff
    // private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //   .withDeadband(Constants.DriveTrain.kMaxMetersPerSecond * 0.1)
    //   .withRotationalDeadband(Constants.DriveTrain.kMaxRotationRadiansPerSecond)
    //   .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //init joysticks
    private final CommandStadiaController dj = new CommandStadiaController(Constants.OI.OI_JOY_DRIVER);
    private final CommandStadiaController oj = new CommandStadiaController(Constants.OI.OI_JOY_OPER);

   /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    disabled().onTrue(new InstantCommand(this::resetRobot).ignoringDisable(true));
    // Configure the button bindings
    configureButtonBindings();
    buildDashboards();

    // Enable closed loop control of compressor and enable it
    // if(Constants.Air.isDisabled) m_air.disable();

    // Enable the camera server and start capture
    if(Constants.Global.CAMERA_ENABLED) {
      // UsbCamera cam = CameraServer.startAutomaticCapture();
      // cam.setResolution(320, 240);
      // cam.setFPS(25);

    }

    // Set the default command that is run for the robot. Normally, this is the drive command
    if(!Constants.DriveTrain.isDisabled) {
      m_drive.setDefaultCommand(
        new drive_defaultDrive(
          m_drive,
          () -> -dj.getLeftY(),
          () -> dj.getLeftX(),
          () -> -dj.getRightX()
        )
      );
    }
  }

  /**
   * This performs robot reset initializations when the disabled() trigger fires.
   */
  private void resetRobot() {
    lighting.init();
    m_aimer.init();
    m_climber.init();
    m_indexer.init();
    m_intake.init();
    m_shooter.init();
  }

  private void configureButtonBindings() {
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

    if(!Constants.DriveTrain.isDisabled) {
      //Uses CTRE SwerveDrive, which requires TalonFX drive+steer and Pigeon2
      // drivetrain.setDefaultCommand( //Drivetrain will execute this command periodically
      //   drivetrain.applyRequest(() -> drive
      //     .withVelocityX(-dj.getLeftY() * Constants.DriveTrain.kMaxMetersPerSecond)
      //     .withVelocityY(dj.getLeftX() * Constants.DriveTrain.kMaxMetersPerSecond)
      //     .withRotationalRate(dj.getRightX() * Constants.DriveTrain.kMaxRotationRadiansPerSecond)
      //   ).ignoringDisable(true));
    }

    /** DRIVER JOYSTICK (dj) */
    // Add random offset to pose estimator to test vision correction
    dj.stadia().onTrue(new InstantCommand(() -> {
      Random rand = new Random(1918);
      var trf = new Transform2d(new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
        new Rotation2d(rand.nextDouble() * 2 * Math.PI));
      m_drive.resetPose(gyro.getHeading().getDegrees(), m_drive.getPose().plus(trf));
    }).ignoringDisable(true));
    // Reset Gyro
    dj.hamburger()
      .onTrue(new gyro_resetGyro().andThen(new drive_resetOdometry(m_drive, new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-180.0)))));
    // Defensive Lock (brake + rotate wheels 45 degrees in an X pattern)
    dj.rightTrigger().whileTrue(new drive_defLock(m_drive));
    dj.a()
      .onTrue(new InstantCommand(() -> lighting.setColor(Colors.NCBLUE)).ignoringDisable(true))
      .onFalse(new InstantCommand(() -> lighting.setColor(Colors.OFF)).ignoringDisable(true));
    dj.b()
      .onTrue(new InstantCommand(() -> lighting.setColor(Colors.NCGREEN)).ignoringDisable(true))
      .onFalse(new InstantCommand(() -> lighting.setColor(Colors.OFF)).ignoringDisable(true));
    /** OPERATOR JOYSTICK (oj) */

  }


  /**
   * Use this to pass the named command to the main Robot class.
   * @return command
   */
  public Command getAutonomousCommand() {
    System.out.println("Shuffle: Selected auton routine is "+m_auto_chooser.getSelected().getName());
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
      case "resetRobot":
        return new cg_resetRobot();
      default:
        return null;
    }
  }

  //From here down is all used for building the shuffleboard
  public void buildDashboards(){
    //List of Widgets: https://github.com/Gold872/elastic-dashboard/wiki/Widgets-List-&-Properties-Reference
    buildAutonChooser();
    buildDriverTab();
    buildMaintenanceTab();
    gyro.buildDashboards();
  }

  public void buildAutonChooser() {
    //This builds the auton chooser, giving driver friendly names to the commands from above
    if(Constants.Auton.isDisabled) {
      m_auto_chooser.setDefaultOption("Do Nothing", new cg_autonDoNothing(m_drive));
    } else {
      m_auto_chooser.setDefaultOption("Do Nothing", new cg_autonDoNothing(m_drive));
    }
    //SmartDashboard.putData(m_auto_chooser); //put in the smartdash
  }

  private void buildDriverTab(){
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    // Match Time - Cannot be programmatically placed, but we put it here for informative reasons
    driverTab.add("Match Time", "")
      .withPosition(0,2)
      .withSize(8,3)
      .withProperties(Map.of("time_display_mode","Minutes and Seconds","red_start_time",15,"yellow_start_time",30)) //mode: "Seconds Only" or "Minutes and Seconds"
      .withWidget("Match Time");
    // Auton Chooser
    driverTab.add("Autonomous Chooser", m_auto_chooser)
      .withPosition(0, 3)
      .withSize(8, 2)
      .withWidget("ComboBox Chooser");
    // FMS Info - Cannot be programmatically placed, but we put it here for informative reasons
    driverTab.add("FMS Info", "")
      .withPosition(0,5)
      .withSize(8,2)
      .withWidget("FMSInfo");
    // Camera
    // driverTab.add("Camera", Robot.camera)
    //   .withPosition(8,0)
    //   .withSize(11,7)
    //   // .withProperties(Map.of("Glyph","CAMERA_RETRO","Show Glyph",true,"Show crosshair",true,"Crosshair color","#CCCCCC","Show controls",false))
    //   .withWidget("Camera Stream");
  }

  private void buildMaintenanceTab(){ //This is where we add maintenance commands
    ShuffleboardTab maintTab = Shuffleboard.getTab("Maintenance");
    // maintTab.add("Reset Robot", new cg_resetRobot(m_stove, m_fsr, m_vision))
    //     .withPosition(0, 0)
    //     .withSize(1, 1);
    // maintTab.add("Zero Robot", new cg_zeroMovingParts(m_stove, m_fsr))
    //     .withPosition(1, 0)
    //     .withSize(1, 1);
}
}
