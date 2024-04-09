
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
 * Constants for the DriveTrain subsystem
 */
public class DriveTrainConstants {
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
    //NCServe Speed Chart with 4" Colson Wheels and Kraken (6000 RPM Free Speed, 5800 RPM with FOC)
    //16:36 = 15.51ft/s or 4.73m/s
    //16:34 = 16.43ft/s or 5.01m/s
    //16:32 = 17.45ft/s or 5.32m/s
    //16:36 = 13.91ft/s or 4.24m/s FOC
    //16:34 = 14.73ft/s or 4.49m/s FOC
    //16:32 = 15.65ft/s or 4.77m/s FOC
    public static final double kMaxMetersPerSecond = 4.77; //limit full stick speed meters per second
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
