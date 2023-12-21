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
    public static final class Spatula {
        public static final void setPositionName(String module, String value) { SmartDashboard.putString("Spatula/"+module+"/PosName", value); }
        public static final void setPosition(String module, double value) { SmartDashboard.putNumber("Spatula/"+module+"/Position", value); }
        public static final void setTarget(String module, double value) { SmartDashboard.putNumber("Spatula/"+module+"/Target",value); }
        public static final void setSpeed(String module, double value) { SmartDashboard.putNumber("Spatula/"+module+"/Speed",value); }
    }
    public static final class Aimer {
        public static final void setPosition(double value) { SmartDashboard.putNumber("Aimer/Position",value); }
        public static final void setTarget(double value) { SmartDashboard.putNumber("Aimer/Target",value); }
        public static final void setError(double value) { SmartDashboard.putNumber("Aimer/Error", value); }
        public static final void setSpeed(double value) { SmartDashboard.putNumber("Aimer/Speed",value); }
    }
    public static final class Griddle {
        public static final void setSpeed(double value) { SmartDashboard.putNumber("Griddle/Speed", value); }
        public static final void setDirection(String value) { SmartDashboard.putString("Griddle/Direction", value); }
    }
    public static final class ConvectionFan {
        public static final void setEnabled(boolean value) { SmartDashboard.putBoolean("Convection Fan", value); }
    }
    public static final class Burner {
        public static final void setPositionName(String value) { SmartDashboard.putString("Burner/PosName", value); }
        public static final void setPosition(double value) { SmartDashboard.putNumber("Burner/Position",value); }
        public static final void setTarget(double value) { SmartDashboard.putNumber("Burner/Target",value); }
        public static final void setError(double value) { SmartDashboard.putNumber("Burner/Error", value); }
        public static final void setSpeed(double value) { SmartDashboard.putNumber("Burner/Speed",value); }
        public static final void setFLimit(boolean value) { SmartDashboard.putBoolean("Burner/FLimit", value); }
        public static final void setRLimit(boolean value) { SmartDashboard.putBoolean("Burner/RLimit", value); }
    }
    public static final class GreaseTrap {
        public static final void setPositionName(String value) { SmartDashboard.putString("GreaseTrap/PosName", value); }
        public static final void setPosition(double value) { SmartDashboard.putNumber("GreaseTrap/Position",value); }
        public static final void setTarget(double value) { SmartDashboard.putNumber("GreaseTrap/Target",value); }
        public static final void setError(double value) { SmartDashboard.putNumber("GreaseTrap/Error", value); }
        public static final void setSpeed(double value) { SmartDashboard.putNumber("GreaseTrap/Speed",value); }
        public static final void setLimit(boolean value) { SmartDashboard.putBoolean("GreaseTrap/Limit", value); }
    }
    public static final class HotPlate {
        public static final void setPositionName(String value) { SmartDashboard.putString("HotPlate/PosName", value); }
        public static final void setPosition(double value) { SmartDashboard.putNumber("HotPlate/Position",value); }
        public static final void setTarget(double value) { SmartDashboard.putNumber("HotPlate/Target",value); }
        public static final void setError(double value) { SmartDashboard.putNumber("HotPlate/Error", value); }
        public static final void setSpeed(double value) { SmartDashboard.putNumber("HotPlate/Speed",value); }
        public static final void setLimit(boolean value) { SmartDashboard.putBoolean("HotPlate/Limit", value); }
    }
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
        // public static final void setCommunity(boolean value) { SmartDashboard.putBoolean("In Community", value); }
        public static final void setCommunity(boolean value) { SmartDashboard.putBoolean("In Community", value); driver_CommunityEntry.setBoolean(value); }
    }

    // Define on-the-fly tabs
    private static ShuffleboardTab tabDriver = Shuffleboard.getTab("Driver");
    private static GenericEntry driver_CommunityEntry = tabDriver.add("Community",false).withPosition(9,0).withSize(1,2).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
}