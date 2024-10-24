/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import java.util.Optional;
import frc.team1918.robot.constants.*; 

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team1918.lib.statefactory.state.StateMachine;
import frc.team1918.lib.statefactory.state.StateMachineBuilder;
import frc.team1918.robot.Helpers.Debug;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {
  public Optional<Alliance> m_alliance;
  private Command m_autonomousCommand;
  private Command m_disableCommand;
  public static UsbCamera camera;
  // private Command m_initOdom;
  // private Command m_resetGyro;

  public StateMachine fsm = null;
  enum States { //States for the FSM
    WAIT, //Waiting for something 
    DONE, //End state
  }

  private RobotContainer m_robotContainer;
  private boolean m_yawOffsetHasBeenSet = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Disable LiveWindow
    LiveWindow.disableAllTelemetry();
    // LiveWindow.enableAllTelemetry();

    //Attempt to get alliance - This can change, so we also get this during periodic
    m_alliance = DriverStation.getAlliance();

    //Silence Joystick Warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    //Setup camera
    camera = CameraServer.startAutomaticCapture();
    camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
    // camera.setResolution(640, 480);
    // camera.setFPS(20);

    //Create forwarder
    //ll3 (photonvision)
    PortForwarder.add(15800,"photonvision.local",5800);
    PortForwarder.add(11181,"photonvision.local",1181);
    PortForwarder.add(11182,"photonvision.local",1182);

    //ll3 (photonvision2)
    PortForwarder.add(25800,"photonvision2.local",5800);
    PortForwarder.add(21181,"photonvision2.local",1181);
    PortForwarder.add(21182,"photonvision2.local",1182);

    //limelight (photovision)
    // PortForwarder.add(5800,"gloworm.local",5800);
    // PortForwarder.add(1181,"gloworm.local",1181);
    // PortForwarder.add(1182,"gloworm.local",1182);
    //limelight (limelightOS)
    // for (int port = 5800; port <= 5807; port++) {
    //   PortForwarder.add(port,"limelight.local",port);
    // }

    //build a finite state machine
    fsm = new StateMachineBuilder()
      .state(States.WAIT)
      .onEnter( () -> {
        //do this when entering the state
      })
      .loop( () -> {
        //repeatedly do this
      })
      .onExit( () -> {
        //do this when leaving the state
      })
      .transition( () -> (true))
      .transitionWithPointerState( () -> (false), States.DONE)
      .state(States.DONE)
      .onEnter( () -> {
        //do this when entering the state
      })
      .loop( () -> {
        // Debug.debug("FSM Done");
      })
      .build();

      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    m_alliance = DriverStation.getAlliance();
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // m_disableCommand = m_robotContainer.getRobotCommand("resetRobot");
    // if (m_disableCommand != null) m_disableCommand.schedule();
    // m_dc1 = m_robotContainer.getDisableCommand(1);
    // if (m_dc1 != null) m_dc1.schedule();
    // m_dc2 = m_robotContainer.getDisableCommand(2);
    // if (m_dc2 != null) m_dc2.schedule();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_alliance = DriverStation.getAlliance(); //Put Alliance.Red or Alliance.Blue in Robot.m_alliance

    //Set the yaw offset based on the current robot pose
    var resetGyro = m_robotContainer.getRobotCommand("yawFromPose");
    if (!m_yawOffsetHasBeenSet && resetGyro != null) { resetGyro.schedule(); m_yawOffsetHasBeenSet=true; }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null && !AutonConstants.isDisabled) m_autonomousCommand.schedule();

    //start the fsm
    // fsm.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //update the fsm
    // fsm.update();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();

    //Set the yaw offset based on the current robot pose
    var resetGyro = m_robotContainer.getRobotCommand("yawFromPose");
    if (!m_yawOffsetHasBeenSet && resetGyro != null) { resetGyro.schedule(); m_yawOffsetHasBeenSet=true; }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // @SuppressWarnings("unused")
  @Override
  public void simulationInit() {
    // NetworkTableInstance nt = NetworkTableInstance.getDefault();
    // nt.stopServer();
    // nt.setServer("10.19.18.12");
    // nt.startClient4("Robot Simulation");

    int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
  }

  // @SuppressWarnings("unused")
  @Override
  public void simulationPeriodic() {
    // int m_simgyro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Yaw"));
    // SimDouble pitch = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_simgyro,"Pitch"));
    // // angle.set(angle.get()+0.25);
    // // pitch.set(pitch.get()+0.25);
  }

}
