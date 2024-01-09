//OI = Operator Interface
package frc.team1918.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class reads and writes values to/from the SmartDashboard
 */
public class Dashboard {
    public static final class Vision {
        public static final void setVisionRinglight(boolean value) { SmartDashboard.putBoolean("Vision/Ring Light", value); }
    }
    public static final class Gyro {
        public static final void setGyroAngle(double value) { SmartDashboard.putNumber("GyroAngle",value); }
        public static final void setGyroPitch(double value) { SmartDashboard.putNumber("GyroPitch", value); }
    }
    public static final class DriveTrain {
        public static final void setTurnPosition(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Position", value); }
        public static final void setTurnSetpoint(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Setpoint", value); }
        public static final void setTurnPositionError(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Position Error", value); }
        public static final void setTurnVelocity(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Turn Velocity", value); }
        public static final void setTurnPositionErrorChange(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Position Error Change", value); }
        public static final void setTurnZeroPosition(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Zero Position", value); }
        public static final void setDriveVelocity(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Drive Velocity", value); }
        public static final void setDriveDistance(String module, double value) { SmartDashboard.putNumber("Swerve/"+module+"/Drive Distance", value); }
        public static final void setHeading(double value) { SmartDashboard.putNumber("Heading", Helpers.General.roundDouble(value,2)); }
        public static final void setX(double value) { SmartDashboard.putNumber("Current X", value); }
        public static final void setY(double value) { SmartDashboard.putNumber("Current Y", value); }
        public static final void setCurrentAngle(double value) { SmartDashboard.putNumber("Current Angle", value); }
        public static final void setTargetAngle(double value) { SmartDashboard.putNumber("Target Angle", value); }
        public static final void setRotationPidOut(double value) { SmartDashboard.putNumber("Rotation PID Out", value); }
        public static final void setDesiredAngle(double value) { SmartDashboard.putNumber("Desired Angle", value); }
        public static final void setCorrectionAngle(double value) { SmartDashboard.putNumber("Correction Omega", value); }
    }

    // Define on-the-fly tabs
    private static ShuffleboardTab tabDriver = Shuffleboard.getTab("Driver");
    private static GenericEntry driver_CommunityEntry = tabDriver.add("Community",false).withPosition(9,0).withSize(1,2).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
}