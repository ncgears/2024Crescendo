
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
 * Constants for the Operator Interface
 * The OI is based on 2 Logitech Controllers, a driver and an operator, setup for swerve drive.
 * The driver left stick controls the forward rate (up/down), and strafe rate (left/right).
 * The driver right stick controls the rotation rate (left/right).
 */
public class OIConstants { //we define the axis' here because they are not bound in robotContainer.
    public static final int OI_JOY_DRIVER = 0; //ID of Driver Joystick
    public static final int OI_JOY_OPER = 1; //ID of Operator Joystick
    public static final double kMinDeadband = 0.1; //Deadband for analog joystick axis minimum
    public static final double kMaxDeadband = 0.95; //Deadband for analog joystick axis minimum
}

