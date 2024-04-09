
package frc.team1918.robot.constants;

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
 * Constants for the Swerve Modules
 */
public class SwerveConstants {
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
    public static final double kMaxSpeedMetersPerSecond = DriveTrainConstants.kMaxMetersPerSecond; 
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
        new Translation2d(Units.inchesToMeters(GlobalConstants.kWheelbaseLength / 2), Units.inchesToMeters(-GlobalConstants.kWheelbaseWidth / 2)),
        new Translation2d(Units.inchesToMeters(GlobalConstants.kWheelbaseLength / 2), Units.inchesToMeters(GlobalConstants.kWheelbaseWidth / 2)),
        new Translation2d(Units.inchesToMeters(-GlobalConstants.kWheelbaseLength / 2), Units.inchesToMeters(-GlobalConstants.kWheelbaseWidth / 2)),
        new Translation2d(Units.inchesToMeters(-GlobalConstants.kWheelbaseLength / 2), Units.inchesToMeters(GlobalConstants.kWheelbaseWidth / 2))
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
        public static final double TURN_kP = SwerveConstants.kDefaultModuleTurnP; //PID P (only change to ovebride default)
        public static final double TURN_kI = SwerveConstants.kDefaultModuleTurnI; //PID I (only change to ovebride default)
        public static final double TURN_kD = SwerveConstants.kDefaultModuleTurnD; //PID D (only change to ovebride default)
        public static final int TURN_kIZone = SwerveConstants.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
        public static final int kTurnAllowedError = SwerveConstants.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
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
        public static final double TURN_kP = SwerveConstants.kDefaultModuleTurnP; //PID P (only change to ovebride default)
        public static final double TURN_kI = SwerveConstants.kDefaultModuleTurnI; //PID I (only change to ovebride default)
        public static final double TURN_kD = SwerveConstants.kDefaultModuleTurnD; //PID D (only change to ovebride default)
        public static final int TURN_kIZone = SwerveConstants.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
        public static final int kTurnAllowedError = SwerveConstants.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
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
        public static final double TURN_kP = SwerveConstants.kDefaultModuleTurnP; //PID P (only change to ovebride default)
        public static final double TURN_kI = SwerveConstants.kDefaultModuleTurnI; //PID I (only change to ovebride default)
        public static final double TURN_kD = SwerveConstants.kDefaultModuleTurnD; //PID D (only change to ovebride default)
        public static final int TURN_kIZone = SwerveConstants.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
        public static final int kTurnAllowedError = SwerveConstants.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
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
        public static final double TURN_kP = SwerveConstants.kDefaultModuleTurnP; //PID P (only change to ovebride default)
        public static final double TURN_kI = SwerveConstants.kDefaultModuleTurnI; //PID I (only change to ovebride default)
        public static final double TURN_kD = SwerveConstants.kDefaultModuleTurnD; //PID D (only change to ovebride default)
        public static final int TURN_kIZone = SwerveConstants.kDefaultModuleTurnIZONE; //PID IZONE (only change to ovebride default)
        public static final int kTurnAllowedError = SwerveConstants.kDefaultModuleTurnAllowableError; //PID Allowed ebror  (only change to ovebride default)
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MC_ID, DRIVE_isInverted, TURN_MC_ID, TURN_sensorPhase, TURN_isInverted, kTurnAllowedError, TURN_kP, TURN_kI, TURN_kD, TURN_kIZone, DRIVE_wheelDiamMM);
    }
}
