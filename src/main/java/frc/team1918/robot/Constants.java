
package frc.team1918.robot;

import frc.team1918.robot.utils.PIDGains;
import frc.team1918.robot.modules.SwerveModuleConstants;
import frc.team1918.robot.utils.TalonConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants let us quickly define the characteristics of our robot without having to search through code
 */
public class Constants {

    /**
     * ID abstracts the IDs from the rest of the constants to make it easier to detect conflicts
     */
    public static final class ID {
        /**
         * IDs of RoboRio Digital IO
         */
        public static final class DIO {
            //public static int some_named_dio = 0;
            public static int indexer_beambreak = 0;
        }
        /**
         * IDs of RoboRio Analog IO
         */
        public static final class Analog {
            //public static int some_named_aio = 0;
        }
        /**
         * IDs of RoboRio PWM
         */
        public static final class PWM {
            public static int climber_leftServo = 0;
            public static int climber_rightServo = 1;
            public static int climber_ratchetServo = 2;
        }
        /**
         * IDs of RoboRio Relays
         */
        public static final class Relay {
        }
        /**
         * IDs of Talons
         */
        public static final class Talon {
            public static int swerve_fl_turn = 1;
            public static int swerve_fr_turn = 4;
            public static int swerve_bl_turn = 2;
            public static int swerve_br_turn = 3;
            public static int intake = 10;
            public static int indexer = 11;
        }
        /**
         * IDs of Falcons
         */
        public static final class Falcon {
            public static int swerve_fl_drive = 5;
            public static int swerve_fr_drive = 8;
            public static int swerve_br_drive = 7;
            public static int swerve_bl_drive = 6;
            public static int shootertop = 12;
            public static int shooterbottom = 13;
            public static int aimer = 14;
            public static int arm = 15;
            public static int climber = 16;
        }
        /**
         * IDs of Krakens
         */
        public static final class Kraken {
        }
        /**
         * IDs of CANdles
         */
        public static final class CANdle {
            public static int candle1 = 17;
            public static int candle2 = 18;
        }
        /**
         * IDs of CANcoders
         */
        public static final class CANcoder {
            public static int aimer = 20;
            public static int climber = 21;
            public static int arm = 22;
        }

    }

    /**
     * Constants that are Global for the robot
     */
    public static final class Global {
        //Global Constants
        public static final int kFalconMaxRPS = 6350 / 60;
        public static final int kKrakenMaxRPS = 6000 / 60;
        public static final boolean CAMERA_ENABLED = false; //set to false if UsbCamera is removed
        public static final boolean SWERVE_SENSOR_NONCONTINUOUS = false;
        public static final int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
        public static final int kPidIndex = 0;  //Talon PID index for primary loop
        public static final int kPidProfileSlotIndex = 0; //PID Profile gains slot
        //2024 robot is 28x31 (frame perim), wheelbase
        // public static final int ROBOT_WIDTH = 28; //Width of the robot frame (from the pivot of the wheels)
        // public static final int ROBOT_LENGTH = 28; //Length of the robot frame (from the pivot of the wheels)
        public static final int kWheelbaseWidth = 23; //from pivot to pivot of swerve module side to side
        public static final int kWheelbaseLength = 23; //from pivot to pivot of the swerve module front to back
        public static final int kFrameWidth = 28; //outside frame perimeter side to side
        public static final int kFrameLength = 30; //outside frame perimeter front to back
        public static final int kBumperWidth = kFrameWidth + 7; //outside of bumpers side to side //.89m
        public static final int kBumperLength = kFrameLength + 7; //outside of bumpers front to back //.965m
        public static final boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
        public static final int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recubring debug messages
        public static final int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
        public final static boolean tuningMode = true; //Enable tunable numbers
    }

    /**
     * Constants for the Autonomous subsystem
     */
    public static final class Auton {
        public static final boolean isDisabled = false; //Disable autonomous
        public static final boolean kUseTracking = true; //enable target tracking during auton pathing
        public static final double kMaxSpeedMetersPerSecond = 5.66;
        public static final double kMaxAccelMetersPerSecondSquared = 1.0;
        public static final double kMaxOmega = 2.0 * Math.PI; //(kMaxSpeedMetersPerSecond / Math.hypot(0.584 / 2.0, 0.66 / 2.0));
        public static final double kPTranslationController = 5; //0.85;
        public static final double kPThetaController = 5; //0.8;
    }

    /**
     * Constants for the Vision class
     */
    public static final class Vision {
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final boolean kUseAutoSuppress = true; //enable suppressing vision measurements based on speed
        public static final double kAutosuppressSpeedMetersPerSecond = 2.5; //speed at which to suppress vision addition
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final class Front { //forward facing camera
            public static final String kCameraName = "llcam1";
            public static final boolean kUseForPose = false;
            public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(0.34,-0.195,0.23), //0.34; x,y,z location of camera on robot in meters
                new Rotation3d(0,Math.toRadians(33),0) //yaw,pitch/roll of camera on robot in radians
            );
        }
        public static final class Back { //backwards facing camera
            public static final String kCameraName = "llcam2";
            public static final boolean kUseForPose = true;
            public static final Transform3d kRobotToCam = new Transform3d(
                new Translation3d(-0.34,0.19,0.23), //-0.34x,y,z location of camera on robot in meters
                new Rotation3d(0,Math.toRadians(33),Math.toRadians(180)) //yaw,pitch/roll of camera on robot in radians
            );
        }
    }

    /**
     * Constants for the Audio Class
     */
    public static final class Audio {
        public static final boolean isEnabled = true;
    }

    /**
     * Constants for the Template Subsystem
     */
    public static final class Template {
    }

    /**
     * Constants for the Lighting Subsystem
     */
    public static final class Lighting {
        public static final String canBus = "rio";
        public static final boolean debugDashboard = false; //enable debugging dashboard
        public static final int kCandle1ID = ID.CANdle.candle1;
        public static final int kCandle2ID = ID.CANdle.candle2;
    }

    public static final class Gyro {
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final boolean kGyroReversed = true;
    }

    /**
     * Constants for the Intake Subsystem
     */
    public static final class Intake {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = false; //enable debugging dashboard
        public static final int kMotorID = ID.Talon.intake;
        public static final boolean kIsInverted = true;
        public static final NeutralMode kNeutralMode = NeutralMode.Brake;
        public static final double kSpeed = 0.75;
    }

    /**
     * Constants for the Intake Subsystem
     */
    public static final class Indexer {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = false; //enable debugging dashboard
        public static final int kMotorID = ID.Talon.indexer;
        public static final boolean kIsInverted = false;
        public static final NeutralMode kNeutralMode = NeutralMode.Brake;
        public static final double kSpeed = 1.0;
        public static final int kBeamBreakID = ID.DIO.indexer_beambreak;
    }
    
    /**
     * Constants for the Aimer Subsystem
     */
    public static final class Aimer {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final int kCANcoderID = ID.CANcoder.aimer;
        public static final boolean kUseCANcoder = true;
        public static final double kMagnetOffset = 0.1623535; //0.17333984375; //Adjust magnet to sensor offset for CANcoder
        public static final int kMotorID = ID.Falcon.aimer;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final double kStowPosition = 0.0;
        public static final double kGearRatio = 5.0; // 16t:80t
        public static final double kPositionThreshold = 0.02; //close enough to target position
        public static final double kGravityDistanceOffset = 1.25; //remove from distance before calculating gravity adjustment
        public static final double kGravityMultiplier = 0.008; //multiplied by distance to get adjustment to aim
        public static final double kDistanceOffset = 0.25; //offset distance for angle calculations
        public static final double kDistanceMinimumForOffset = 0.0;
        //PID Control
        public static final double kS = 0.10; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 40.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.0; // no integral
        public static final double kD = 0.01; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 10; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 10; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 2000; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        // Review https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        // Using motionMagic: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html

        // public static final double kPeakFwdVoltage = 9.0;
        // public static final double kPeakRevVoltage = -9.0;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false;
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        //Ramping (0.0 by default)
        public static final double kOpenLoopRamp = 0.0;
		public static final double kClosedLoopRamp = 0.0;
        public class Positions {
            public static final double kTrapClimb = -0.1030273;
            public static final double kRevLimit = -0.104;
            public static final double kFwdLimit = 0.168;
            public static final double kShotShort = 0.1620;
            public static final double kShotLong = 0.1010;
        }
        public static final boolean kSoftForwardLimitEnable = true;
        public static final double kSoftForwardLimit = Positions.kFwdLimit;
        public static final boolean kSoftReverseLimitEnable = true;
        public static final double kSoftReverseLimit = Positions.kRevLimit;
    }

    /**
     * Constants for the Arm Subsystem
     */
    public static final class Arm {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = false; //enable debugging dashboard
        public static final int kCANcoderID = ID.CANcoder.arm;
        public static final boolean kUseCANcoder = true;
        public static final double kMagnetOffset = -0.098876953125; //Adjust magnet to sensor offset for CANcoder
        public static final int kMotorID = ID.Falcon.arm;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        public static final double kStowPosition = 0;
        public static final double kGearRatio = 49.08163; //9.816326; // 14t:26t -> 14t:74t
        public static final double kPositionThreshold = 0.04; //close enough to target position
        //PID Control
        public static final double kS = 0.15; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 24.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.0; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 80; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 1600; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false;
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        //Positions
        public class Positions {
            public static final double kIntake = 0.0;
            public static final double kTrapBalance = 0.2751465; //position for balancing for trap climb
            public static final double kTrapClimb = 0.0932617; //position to prepare for trap climbing
            public static final double kAmp = 0.381; //amp scoring position
            public static final double kTrap = 0.345; //position to put note in trap
        }
        public static final boolean kSoftForwardLimitEnable = true;
        public static final double kSoftForwardLimit = 0.4;
        public static final boolean kSoftReverseLimitEnable = true;
        public static final double kSoftReverseLimit = -0.01;
    }

    /**
     * Constants for the Climber Subsystem
     */
    public static final class Climber {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = false; //enable debugging dashboard
        public static final boolean isDisabled = false; //disable climber default command
        public static final int kCANcoderID = ID.CANcoder.climber;
        public static final boolean kUseCANcoder = true;
        public static final double kMagnetOffset = -0.6903906; //Adjust magnet to sensor offset for CANcoder
        public static final int kMotorID = ID.Falcon.climber;
        public static final boolean kIsInverted = true;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kStowPosition = 0;
        public static final int kLeftServoID = ID.PWM.climber_leftServo;
        public static final int kRightServoID = ID.PWM.climber_rightServo;
        public static final int kRatchetServoID = ID.PWM.climber_ratchetServo;
        // public static final double kGearRatio = 26.67; // 12:1 gearbox, 18t:42t -- this is between rotor and sensor
        public static final double kGearRatio = 46.667; // 20:1 gearbox (0.05), 18t:42t -- this is between rotor and sensor
        public static final double kSensorGearRatio = 1.0; // no gearing between sensor and spool -- this is between sensor and spool
        //PID Control
        public static final double kS = 0.22; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.0; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.0; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 32.0; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.01; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 30; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 200; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 0; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false; // TODO: Test current limits
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        public class Positions {
            public static final double kArmLimit = 1.5239258; //max height of climber when arm swings over
            public static final double kTop = 3.38; //max height (entire travel 3.392)
            public static final double kFwdLimit = 3.38; //absolute limit
            public static final double kBottom = 0.0; //all the way down
            public static final double kTopHookCapture = 1.401123; //robot climber to capture top hook
            public static final double kTopHookClimb = 0.0; //robot off the ground using top hook
            public static final double kMidHookClear = kTop; //drive mid hook onto chain
            public static final double kLatchClimb = 0.0; //chain is on latches
            public static final double kLatchClear = 0.0; //chain is below latches
        }
        public static final boolean kSoftForwardLimitEnable = true;
        public static final double kSoftForwardLimit = Positions.kFwdLimit;
        public static final boolean kSoftReverseLimitEnable = true;
        public static final double kSoftReverseLimit = -0.01;
    }

    /**
     * Constants for the Shooter Subsystem
     */
    public static final class Shooter {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast; 
        public static final boolean isInverted = false;
        public static final double kGearRatio = 0.66667; //36:24 pulley //Adjust for gearing on output of Falcon
        public static final double kMaxRPS = Global.kFalconMaxRPS / kGearRatio; //The Maximum free speed of the shooter
        public static final double kSpeedToleranceOptimal = 2.0; //How close in RPS is considered at speed
        public static final double kSpeedToleranceAcceptable = 5.0; //How close in RPS is considered close enough to shoot
        //PID Control
        //these are based on SysId characterization with 1:1 pulleys on 3/17/2024
        // public static final double kS = 0.1367; // add kS to overcome static friction: adjust first to start moving
        // public static final double kV = 0.12189; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        // public static final double kA = 0.0080711; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        // public static final double kP = 0.18331; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        //these are based on SysId characterization with 1.5:1 pulleys on 3/21/2024
        public static final double kS = 0.091751; // add kS to overcome static friction: adjust first to start moving
        public static final double kV = 0.085922; // add kV for velocity target: voltage(12) / velocity target.. 1 rps results in 0.12v output
        public static final double kA = 0.010311; // add kA for acceleration: 0.01 = 1 rps/s requires 0.01v output
        public static final double kP = 0.10799; // add kP per rotation of error: error of 1 rotation results in 12v output (this might be low for aimer)
        public static final double kI = 0.0; // no integral
        public static final double kD = 0.0; // 0.1 = velocity error of 1rps results in 0.1v output
        public static final double kMotionMagicCruise = 40; // Motor Max / Gear Ratio
        public static final double kMotionMagicAccel = 400; // Acceleration: Cruise / Accel = time to cruise
        public static final double kMotionMagicJerk = 7000; //0=disabled; 10-20x accel for smooth; lower for smoother motion at the cost of time: accel / jerk = jerk time
        //Current Limiting
        public static final double kPeakFwdVoltage = 12.0;
        public static final double kPeakRevVoltage = -12.0;
        public static final boolean kCurrentLimitEnable = false;
        public static final double kCurrentLimitAmps = 10.0;
        public static final double kCurrentLimitThresholdAmps = 15.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        //Ramping (0.0 by default)
        public static final double kOpenLoopRamp = 0.0;
		public static final double kClosedLoopRamp = 0.0;
        public static final Transform3d kRobotToShooter = new Transform3d(
            new Translation3d(0.0,0.0,0.32), //x,y,z location of shooter on robot in meters
            new Rotation3d(Math.PI,0,0) //yaw,pitch,roll of shooter on robot in radians
        );
        public static final class Top {
            public static final int kMotorID = ID.Falcon.shootertop;
            public static final boolean kIsInverted = false;
        }
        public static final class Bottom {
            public static final int kMotorID = ID.Falcon.shooterbottom;
            public static final boolean kIsInverted = true;
        }
    }
    
    /**
     * Constants for the Swerve Modules
     */
    public static final class Swerve {
        public static final String canBus = "rio"; //name of canbus for swerve modules, if not "rio"
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final boolean homeOnInit = true; //true to go to zero position on init
        public static final boolean useTurnOptimization = true; //This will reduce wear on wheels by only turning <180 and reversing the drive power if appropriate
        // turn pid defaults (used in module definitions)
        public static final double kDefaultModuleTurnP = 2.8; //PID P
        public static final double kDefaultModuleTurnI = 0.0; //PID I
        public static final double kDefaultModuleTurnD = 0.0; //PID D
        public static final int kDefaultModuleTurnIZONE = 0; //PID IZone
        public static final int kDefaultModuleTurnAllowableError = 3; //PID Allowed Ebror
        public static final double kDefaultModuleWheelDiamMM = 101.6; //Wheel Diameter of 4in colson
        public static final int kTurnEncoderFullRotation = 1024; //lamprey2 //was 1023, why?
        public static final double kTurnGearRatio = 10.3846154; //The output of the turn gearbox turns 10 times for one module rotation
        public static final double kRotationsPerWheelRotation = 6.0; //(32/16*45/15); or 6.75 //(44/16*45/15)
        public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
        // Current limits
        // Swerve Current limiting -- Needs tuning, this was bobrowed from Team364 example
        // See {@link https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/CTREConfigs.java}
        public static final boolean kTurnEnableCurrentLimit = true;
        public static final int kTurnCurrentLimitAmps = 25;
        public static final int kTurnCurrentThresholdAmps = 40;
        public static final double kTurnCurrentThresholdSecs = 0.1;
        public static final boolean kDriveCurrentLimitEnabled = true;
        public static final int kDriveCurrentLimitAmps = 38;
        public static final int kDriveCurrentThresholdAmps = 45;
        public static final double kDriveCurrentThresholdSecs = 0.3;
        public static final boolean kDriveStatorCurrentLimitEnable = false;
        public static final int kDriveStatorCurrentLimitAmps = 100;

        /*
		 * These values are used by the drive falcon to ramp in open loop and closed
		 * loop driving.
		 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
		 */
		public static final double kOpenLoopRamp = 0.25;
		public static final double kClosedLoopRamp = 0.0;

        // swerve control definitions
        public static final double kHomeOffsetRadians = 0.0; //3 * (Math.PI/4); //135 - radians to offset the zero point of the wheels
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI; // 540 deg/sec
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI; //720 deg/sec^2
        public static final double kMaxSpeedMetersPerSecond = DriveTrain.kMaxMetersPerSecond; 
        public static final double kDriveKP = 0.05;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        // Drive Motor Characterization
        // How do we determine these numbers? Need to find out. These falcon numbers are from Team364 example
        public static final double kDriveKS = (0.667 / 12); //Static Gain //divide by 12 to convert from volts to percent output for CTRE
        public static final double kDriveKV = (2.44 / 12); //Velocity Gain
        public static final double kDriveKA = (0.27 / 12); //Acceleration Gain

        //Forward Positive, Left Positive, Up Positive (NWU Convention)
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(Global.kWheelbaseLength / 2), Units.inchesToMeters(-Global.kWheelbaseWidth / 2)),
            new Translation2d(Units.inchesToMeters(Global.kWheelbaseLength / 2), Units.inchesToMeters(Global.kWheelbaseWidth / 2)),
            new Translation2d(Units.inchesToMeters(-Global.kWheelbaseLength / 2), Units.inchesToMeters(-Global.kWheelbaseWidth / 2)),
            new Translation2d(Units.inchesToMeters(-Global.kWheelbaseLength / 2), Units.inchesToMeters(Global.kWheelbaseWidth / 2))
        );
        /**
         * Constants for Front Left Swerve Module
         */
        public static final class FL {
            public static final boolean isDisabled = false;
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_fl_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = kDefaultModuleWheelDiamMM; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = ID.Talon.swerve_fl_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is cobrect, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.kDefaultModuleTurnP; //PID P (only change to ovebride default)
            public static final double TURN_kI = Swerve.kDefaultModuleTurnI; //PID I (only change to ovebride default)
            public static final double TURN_kD = Swerve.kDefaultModuleTurnD; //PID D (only change to ovebride default)
            public static final int TURN_kIZone = Swerve.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
            public static final int kTurnAllowedError = Swerve.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, kTurnAllowedError, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Front Right Swerve Module
         */
        public static final class FR {
            public static final boolean isDisabled = false; 
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_fr_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = kDefaultModuleWheelDiamMM; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = true;
            public static final int TURN_MC_ID = ID.Talon.swerve_fr_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is cobrect, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.kDefaultModuleTurnP; //PID P (only change to ovebride default)
            public static final double TURN_kI = Swerve.kDefaultModuleTurnI; //PID I (only change to ovebride default)
            public static final double TURN_kD = Swerve.kDefaultModuleTurnD; //PID D (only change to ovebride default)
            public static final int TURN_kIZone = Swerve.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
            public static final int kTurnAllowedError = Swerve.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, kTurnAllowedError, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Rear Left Swerve Module
         */
        public static final class BL {
            public static final boolean isDisabled = false;
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_bl_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = kDefaultModuleWheelDiamMM; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = ID.Talon.swerve_bl_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is cobrect, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.kDefaultModuleTurnP; //PID P (only change to ovebride default)
            public static final double TURN_kI = Swerve.kDefaultModuleTurnI; //PID I (only change to ovebride default)
            public static final double TURN_kD = Swerve.kDefaultModuleTurnD; //PID D (only change to ovebride default)
            public static final int TURN_kIZone = Swerve.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
            public static final int kTurnAllowedError = Swerve.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, kTurnAllowedError, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Rear Right Swerve Module
         */
        public static final class BR { //Rear Right
            public static final boolean isDisabled = false;
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_br_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = kDefaultModuleWheelDiamMM;  //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = true;
            public static final int TURN_MC_ID = ID.Talon.swerve_br_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is cobrect, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.kDefaultModuleTurnP; //PID P (only change to ovebride default)
            public static final double TURN_kI = Swerve.kDefaultModuleTurnI; //PID I (only change to ovebride default)
            public static final double TURN_kD = Swerve.kDefaultModuleTurnD; //PID D (only change to ovebride default)
            public static final int TURN_kIZone = Swerve.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
            public static final int kTurnAllowedError = Swerve.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, kTurnAllowedError, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
    }

    /**
     * Constants for the DriveTrain subsystem
     */
    public static final class DriveTrain {
        public static final boolean isDisabled = false; 
        public static final class thetaController {
            public static final boolean isEnabled = true;
            public static final double kP = 0.02; //0.02
            public static final double kI = 0.0001; //0.0001
            public static final double kD = 0.002; //0.002
            public static final double kIZone = 2.0;
            public static final double kToleranceDegrees = 0.5;
        }
        public static final class trackingController {
            public static final boolean isEnabled = true;
            public static final double kP = 0.03; //0.02
            public static final double kI = 0.0018; //0.0001
            public static final double kD = 0.001; //0.002
            public static final double kIZone = 2.0;
            public static final double kToleranceDegrees = 1.0;
        }
        public static final class xController {
            public static final double kP = 0.05;
            public static final double kI = 0.0001;
            public static final double kD = 0.002;
        }
        public static final class yController {
            public static final double kP = 0.05;
            public static final double kI = 0.0001;
            public static final double kD = 0.002;
        }
        public static final boolean useDriveStraight = true; //use driveStraight (heading lock)
        public static final boolean useFieldCentric = true; //use field-centric drive. This should always be true except for testing?
        public static final boolean useDefensiveLock = false; //use defensiveLock strategy when braking putting swerve into X pattern
        public static final boolean useBrakeWhenStopped = false; //set the brake mode when drive speed is 0
        //NCServe Speed Chart with 4" Colson Wheels and Falcon500 (6380 RPM Free Speed)
        //16:44 = 13.5fps or 4.11mps
        //16:40 = 14.85fps or 4.53mps
        //16:36 = 16.5fps or 5.03mps
        //16:34 = 17.47fps or 5.32mps
        //16:32 = 18.56fps or 5.66mps
        public static final double kMaxMetersPerSecond = 5.66; //limit full stick speed meters to 13.5fps
        public static final double kMaxRotationRadiansPerSecond = 2 * Math.PI; //3.4; //Multiplier for omega of turning the robot
        ////Drive Tuning
        public static final boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public static final double DT_WHEEL_DIAM_MM = 101.6; //diameter of drive wheels in millimeters
        public static final int DT_DRIVE_ENCODER_FULL_ROTATION = 2048; //falcon integrated encoder is 2048
        //Falcon500 = 6380RPM  free speed : 945RPM Calculated
        public static final int DT_DRIVE_FIRST_GEARONE = 16; //swerve drive first gear set input teeth
        public static final int DT_DRIVE_FIRST_GEARTWO = 36; //swerve drive first gear set output teeth
        public static final int DT_DRIVE_SECOND_GEARONE = 15; //swerve drive second gear set input teeth
        public static final int DT_DRIVE_SECOND_GEARTWO = 45; //swerve drive second gear set output teeth
        public static final double DT_DRIVE_CONVERSION_FACTOR = 0.148148; //first_gearone / first_geartwo * second_gearone / second_geartwo
        // public static final double DT_DRIVE_CONVERSION_FACTOR = (DT_DRIVE_FIRST_GEARONE / DT_DRIVE_FIRST_GEARTWO) * (DT_DRIVE_SECOND_GEARONE / DT_DRIVE_SECOND_GEARTWO); //Conversion factor to cobrect RPM from SparkMax getVelocity()
    }
    
    /** Constants for the Dashboard Interface */
    public class Dashboard {
        public class Colors {
            public static final String NCGREEN = "#00B50F"; //approximate
            public static final String NCBLUE = "#0077B5";
            public static final String RED = "#F44336";
            public static final String GREEN = "#4CAF50";
            public static final String BLUE = "#0000FF";
            public static final String ORANGE = "#FFA500";
            public static final String BLACK = "#000000";
        }
    }

    /**
     * Constants for the Operator Interface
     * The OI is based on 2 Logitech Controllers, a driver and an operator, setup for swerve drive.
     * The driver left stick controls the forward rate (up/down), and strafe rate (left/right).
     * The driver right stick controls the rotation rate (left/right).
     */
    public class OI { //we define the axis' here because they are not bound in robotContainer.
        public static final int OI_JOY_DRIVER = 0; //ID of Driver Joystick
        public static final int OI_JOY_OPER = 1; //ID of Operator Joystick
        public static final double kMinDeadband = 0.1; //Deadband for analog joystick axis minimum
        public static final double kMaxDeadband = 0.95; //Deadband for analog joystick axis minimum
        public static final RampingStrength kRampingStrength = RampingStrength.LOW;
        public static enum RampingStrength {
            NONE(1.0), //linear inputs, no ramping
            LOW(1.5), //mild ramping
            MEDIUM(2.0), //squared inputs
            HIGH(3.0); //cubic inputs
            private final double exp;
            RampingStrength(double exp) { this.exp = exp; }
            public final double exponent() { return this.exp; }
        }

        /**
         * This class defines the hardware button and axis IDs for a Logitech F310 Controller.
         * The buttons abray is 1-based, but the axis abray is 0-based
         */
        public static final class Logitech {
            //DO NOT EDIT THESE
            static final int BTN_A = 1; //A Button
            static final int BTN_B = 2; //B Button
            static final int BTN_X = 3; //X Button
            static final int BTN_Y = 4; //Y Button
            static final int BTN_LB = 5; //Left Bumper (L1)
            static final int BTN_RB = 6; //Right Bumper (R1)
            static final int BTN_BACK = 7; //Back Button (Select)
            static final int BTN_START = 8; //Start Button
            static final int BTN_L = 9; //Left Stick Press (L3)
            static final int BTN_R = 10; //Right Stick Press (R3)
            static final int AXIS_LH = 0; //Left Analog Stick horizontal
            static final int AXIS_LV = 1; //Left Analog Stick vertical
            static final int AXIS_LT = 2; //Analog Left Trigger
            static final int AXIS_RT = 3; //Analog Right Trigger
            static final int AXIS_RH = 4; //Right Analog Stick horizontal
            static final int AXIS_RV = 5; //Right Analog Stick vertical
            static final int DPAD_UP = 0;
            static final int DPAD_UPRIGHT = 45;
            static final int DPAD_RIGHT = 90;
            static final int DPAD_DNRIGHT = 135;
            static final int DPAD_DN = 180;
            static final int DPAD_DNLEFT = 225;
            static final int DPAD_LEFT = 270;
            static final int DPAD_UPLEFT = 315;
            static final int DPAD_IDLE = -1; 
        }
    }
}
