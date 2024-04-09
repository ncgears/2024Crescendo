
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
 * Constants for the Aimer Subsystem
 */
public class AimerConstants {
    //Controller Setup
    public static final String canBus = "rio";
    public static final boolean debugDashboard = true; //enable debugging dashboard
    public static final int kCANcoderID = ID.CANcoder.aimer;
    public static final boolean kUseCANcoder = true;
    public static final double kMagnetOffset = -0.158; //0.1623535; //Adjust magnet to sensor offset for CANcoder
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
