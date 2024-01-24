
package frc.team1918.robot;

import frc.team1918.robot.utils.PIDGains;
import frc.team1918.robot.modules.SwerveModuleConstants;
import frc.team1918.robot.utils.TalonConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
        }
        /**
         * IDs of RoboRio Analog IO
         */
        public static final class Analog {
            //public static int some_named_aio = 0;
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
            public static int swerve_fr_turn = 2;
            public static int swerve_rl_turn = 3;
            public static int swerve_rr_turn = 4;
        }
        /**
         * IDs of Falcons
         */
        public static final class Falcon {
            public static int swerve_fl_drive = 31;
            public static int swerve_fr_drive = 32;
            public static int swerve_rr_drive = 34;
            public static int swerve_rl_drive = 33;
            public static int shooter = 41;
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
            public static int candle1 = 1;
        }

    }

    /**
     * Constants that are Global for the robot
     */
    public static final class Global {
        //Global Constants
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
        public static final int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recurring debug messages
        public static final int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
        public final static boolean tuningMode = true; //Enable tunable numbers
    }

    /**
     * Constants for the Air subsystem
     */
    public static final class Air {
        public static final boolean isDisabled = true; //Disable Air
    }

    /**
     * Constants for the Autonomous subsystem
     */
    public static final class Auton {
        public static final boolean isDisabled = false; //Disable autonomous
        // Auton is defined in robot.java
        // public static final String autonToRun = "auton_BasicShootingAuto"; //4BallAuto, BasicDriveAuto, BasicShootingAuto, None //Name of the auton to run (these are in the bottom of RobotContainer)
        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelMetersPerSecondSquared = 0.1;
        public static final double kMaxOmega = (kMaxSpeedMetersPerSecond / Math.hypot(0.584 / 2.0, 0.66 / 2.0));
        public static final double kPTranslationController = 0.0;
        public static final double kPThetaController = 0.0;
        public static final class Balance {
            public static final boolean kUseDefensiveLock = true; //lock the drive train into defensive position when finished balancing
            public static final double kToleranceDegrees = 3.25; //degrees of tolerance for balancing
            public static final double kP = 0.011;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
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
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final int kCandleID = ID.CANdle.candle1;
    }

    /**
     * Constants for the Shooter Subsystem
     */
    public static final class Shooter {
        //Controller Setup
        public static final String canBus = "rio";
        public static final boolean debugDashboard = true; //enable debugging dashboard
        public static final int kMotorID = ID.Falcon.shooter;
        public static final boolean neutralIsBrake = false; 
        public static final boolean isInverted = false;
        public static final double kGearRatio = 1.0; //Adjust for gearing on output of Falcon
        public static final double kMaxRPS = 6000 * kGearRatio / 60; //The Maximum free speed of the shooter
        public static final double kP = 0.11; //PID P // error of 1 rotation per second result in 2V output
        public static final double kI = 0.0; //PID I // error of 1 rotation per second increases output by 0.5V every second
        public static final double kD = 0.0; //PID D // change of 1 rotation per second squared results in 0.01V output
        public static final double kF = 0.12; //PID F // Falcon500 is 500kV motor, 500rpm per V = 8.33 rps per V, 1/8.33 = 0.12 V per rotation per second
        public static final double kPeakFwdVoltage = 8.0;
        public static final double kPeakRevVoltage = -8.0;
        //Current Limiting
        public static final boolean kCurrentLimitEnable = false;
        public static final double kCurrentLimitAmps = 30.0;
        public static final double kCurrentLimitThresholdAmps = 60.0;
        public static final double kCurrentLimitThresholdSecs = 0.3;
        //Ramping (0.0 by default)
        public static final double kOpenLoopRamp = 0.0;
		public static final double kClosedLoopRamp = 0.0;

        public static final class Top { //This would be used for Talon based conroller
            public static final int kMotorID = ID.Falcon.shooter; //TalonSRX Motor Controller ID
            public static final boolean kSensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final int kSensorTicks = 4096;
            public static final boolean kSensorNotContinuous = false;
            public static final boolean kIsInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final int kAllowedError = 5; //PID Allowed error
            public static final TalonConstants constants = new TalonConstants(kMotorID, kSensorPhase, kSensorTicks, kSensorNotContinuous, kIsInverted, kAllowedError);
            //PID Setup
            public static final double kP = 0.5; //PID P 
            public static final double kI = 0.0; //PID I
            public static final double kD = 0.0; //PID D
            public static final double kF = 0.25; //PID F
            public static final int kIZone = 0; //PID IZONE
            public static final double kPeakOutput = 1.0;
            public static final double kNeutralDeadband = 0.001; //0.04 default
            public static final double kCruise = 4000; //MotionMagic Cruise
            public static final double kAccel = 5000; //MotionMagic Acceleration
            public static final int kSCurve = 0; //MotionMagic SCurve
            public static final PIDGains gains = new PIDGains(kP,kI,kD,kF,kIZone,kPeakOutput,kNeutralDeadband,kCruise,kAccel,kSCurve);
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
        public static final double DEFAULT_TURN_P = 2.8; //PID P
        public static final double DEFAULT_TURN_I = 0.0; //PID I
        public static final double DEFAULT_TURN_D = 0.0; //PID D
        public static final int DEFAULT_TURN_IZONE = 0; //PID IZone
        public static final int DEFAULT_TURN_ALLOWED_ERROR = 3; //PID Allowed Error
        public static final double DEFAULT_WHEEL_DIAM_MM = 101.6; //Wheel Diameter of 3in colson
        public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
        // current limits
        // Swerve current limiting //TODO: Needs tuning, this was borrowed from Team364 example
        // See {@link https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/CTREConfigs.java}
        public static final boolean kTurnEnableCurrentLimit = true;
        public static final int kTurnCurrentLimitAmps = 25;
        public static final int kTurnCurrentThresholdAmps = 40;
        public static final double kTurnCurrentThresholdSecs = 0.1;
        public static final boolean isDriveCurrentLimitEnabled = false;
        public static final int kDriveCurrentLimitAmps = 35;
        public static final int kDriveCurrentThresholdAmps = 60;
        public static final double kDriveCurrentThresholdSecs = 0.3;

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
        public static final boolean kGyroReversed = false;
        public static final double kDriveKP = 0.05;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKF = 0.0;
        // Drive Motor Characterization
        // See {@link https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/controller/SimpleMotorFeedforward.html}
        // How do we determine these numbers? Need to find out. These falcon numbers are from Team364 example
        public static final double driveKS = (0.667 / 12); //Static Gain //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12); //Velocity Gain
        public static final double driveKA = (0.27 / 12); //Acceleration Gain

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
            public static final double DRIVE_wheelDiamMM = 98.9; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = true;
            public static final int TURN_MC_ID = ID.Talon.swerve_fl_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Front Right Swerve Module
         */
        public static final class FR {
            public static final boolean isDisabled = false; 
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_fr_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = 97.8; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = ID.Talon.swerve_fr_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Rear Left Swerve Module
         */
        public static final class RL {
            public static final boolean isDisabled = false;
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_rl_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = 98.7; //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = true;
            public static final int TURN_MC_ID = ID.Talon.swerve_rl_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
        /**
         * Constants for Rear Right Swerve Module
         */
        public static final class RR { //Rear Right
            public static final boolean isDisabled = false;
            public static final int DRIVE_MC_ID = ID.Falcon.swerve_rr_drive; //Falcon500 Motor Controller ID
            public static final double DRIVE_wheelDiamMM = 98.6;  //actual diameter of larger wheel in mm
            public static final boolean DRIVE_isInverted = false;
            public static final int TURN_MC_ID = ID.Talon.swerve_rr_turn; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = Swerve.DEFAULT_TURN_P; //PID P (only change to override default)
            public static final double TURN_kI = Swerve.DEFAULT_TURN_I; //PID I (only change to override default)
            public static final double TURN_kD = Swerve.DEFAULT_TURN_D; //PID D (only change to override default)
            public static final int TURN_kIZone = Swerve.DEFAULT_TURN_IZONE; //PID IZONE (only change to override default)
            public static final int TURN_ALLOWED_ERROR = Swerve.DEFAULT_TURN_ALLOWED_ERROR; //PID Allowed error  (only change to override default)
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, TURN_ALLOWED_ERROR, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
        }
    }

    /**
     * Constants for the DriveTrain subsystem
     */
    public static final class DriveTrain {
        public static final boolean isDisabled = false; 
        ////Global Tuning
        public static final class DriveStraight {
            public static final boolean isDisabled = false; //disable drivestraight function
            public static final double kP = 0.05;
            public static final double kI = 0.0001;
            public static final double kD = 0.002;
        }
        public static final boolean useFieldCentric = true; //use field-centric drive. This should always be true except for testing?
        public static final boolean useDefensiveLock = false; //use defensiveLock strategy when braking putting swerve into X pattern
        public static final boolean useBrakeWhenStopped = false; //set the brake mode when drive speed is 0
        public static final double kDriveStraight_P = 0.0075; //kP for driveStraight correction
        //NCServe Speed Chart with 4" Colson Wheels and Falcon500 (6380 RPM Free Speed)
        //16:44 = 13.5fps or 4.11mps
        //16:40 = 14.85fps or 4.53mps
        //16:36 = 16.5fps or 5.03mps
        //16:34 = 17.47fps or 5.32mps
        //16:32 = 18.56fps or 5.66mps
        public static final double kMaxMetersPerSecond = 5.66; //limit full stick speed meters to 13.5fps
        public static final double kMaxRotationRadiansPerSecond = 3 * Math.PI; //3.4; //Multiplier for omega of turning the robot
        //from swerve
        // public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI; // 540 deg/sec
        // public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI; //720 deg/sec^2
        ////Turn Tuning
        public static final double DT_TURN_MULT_STATIONARY = 1.3; //Turn speed multiplier while not moving
        public static final double DT_TURN_MULT_MOVING = 1.3; //Turn speed multiplier while moving
        public static final boolean DT_TURN_MULT_BEFORE_DB = true; //Apply turn multiplier before deadband
        public static final int DT_TURN_ENCODER_FULL_ROTATION = 1023; //This is for the lamprey2, not the integrated SRX mag encoder (lamprey1 is 1023)
        public static final int kTurnEncoderFullRotation = 4096; //This is for the integrated SRX mag encoder in the gearboxes, not the lamprey
        public static final double kTurnGearRatio = 10.3846154; //The output of the turn gearbox turns 10 times for one module rotation
        ////Drive Tuning
        public static final double DT_FWD_MULT = 1.0; //Fwd throttle multiplier
        public static final double DT_STR_MULT = 1.0; //Str throttle multiplier
        public static final boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public static final double DT_WHEEL_DIAM_MM = 101.6; //diameter of drive wheels in millimeters
        public static final int DT_DRIVE_ENCODER_FULL_ROTATION = 2048; //falcon integrated encoder is 2048
        //Falcon500 = 6380RPM  free speed : 945RPM Calculated
        public static final int DT_DRIVE_FIRST_GEARONE = 16; //swerve drive first gear set input teeth
        public static final int DT_DRIVE_FIRST_GEARTWO = 36; //swerve drive first gear set output teeth
        public static final int DT_DRIVE_SECOND_GEARONE = 15; //swerve drive second gear set input teeth
        public static final int DT_DRIVE_SECOND_GEARTWO = 45; //swerve drive second gear set output teeth
        public static final double DT_DRIVE_CONVERSION_FACTOR = 0.148148; //first_gearone / first_geartwo * second_gearone / second_geartwo
        // public static final double DT_DRIVE_CONVERSION_FACTOR = (DT_DRIVE_FIRST_GEARONE / DT_DRIVE_FIRST_GEARTWO) * (DT_DRIVE_SECOND_GEARONE / DT_DRIVE_SECOND_GEARTWO); //Conversion factor to correct RPM from SparkMax getVelocity()
    }
    
    public static final class Vision {
        public static final String limelightName = "limelight"; //name of the limelight
        public static final boolean isDisabled = false;
        public static final boolean stateLightOn = true;
        public static final double kErrorCorrection_P = 0.65; //Proportional value for multiplying vision angle correction
        public static final double kTurnP = 0.17;
        public static final double kTurnD = 0.0;
        public static final double kOffsetDegrees = 0.0; //Manual offset adjustment; +right; -left
        public static final double kMinTurnPower = 0.25; //Minimum power for turning during vision
        public static final double kCloseEnough = 0.04; //Percentage of allowed error
    }
    /**
     * Constants for the Operator Interface
     * The OI is based on 2 Logitech Controllers, a driver and an operator, setup for swerve drive.
     * The driver left stick controls the forward rate (up/down), and strafe rate (left/right).
     * The driver right stick controls the rotation rate (left/right).
     */
    public static final class OI { //we define the axis' here because they are not bound in robotContainer.
        public static final int OI_JOY_DRIVER = 0; //ID of Driver Joystick
        public static final int OI_JOY_OPER = 1; //ID of Operator Joystick
        public static final double OI_JOY_MIN_DEADBAND = 0.1; //Deadband for analog joystick axis minimum
        public static final double OI_JOY_MAX_DEADBAND = 0.9; //Deadband for analog joystick axis minimum

        /**
         * This class defines the hardware button and axis IDs for a Logitech F310 Controller.
         * The buttons array is 1-based, but the axis array is 0-based
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
