
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
 * Constants for the Climber Subsystem
 */
public class ClimberConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = false; //enable debugging dashboard
    public static final boolean isDisabled = true; //disable climber default command
    public static final int kCANcoderID = ID.CANcoder.climber;
    public static final boolean kUseCANcoder = true;
    public static final double kMagnetOffset = -0.6903906; //Adjust magnet to sensor offset for CANcoder
    public static final int kMotorID = ID.TalonFX.climber;
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
