package frc.team1918.robot.modules;

public class SwerveModuleConstants {
    //turn
    public final int idTurnMotor;
    public final boolean turnSensorPhase;
    public final boolean turnIsInverted;
    public final int turnMaxAllowedError;
    public final double turnP;
    public final double turnI;
    public final double turnD;
    public final int turnIZone;
    //drive
    public final int idDriveMotor;
    public final boolean driveIsInverted;
    public final double driveWheelDiam;

    /**
     * Creates a new constants object for initializing a 1918 swerve module v3.0
     * @param idDriveMotor - CAN ID of the Drive Motor (Falcon500)
     * @param driveIsInverted - true to invert the drive directon
     * @param idTurnMotor - CAN ID of the Turn Motor (TalonSRX/Bag)
     * @param turnSensorPhase - true to update the sensor phase to match the encoder
     * @param turnIsInverted - true to invert the turn direction so forward is green (after sensor phase is matched)
     * @param turnMaxAllowedError - maximum allowable error
     * @param turnP - Proportional constant for the turn controller
     * @param turnI - Integral constant for the turn controller
     * @param turnD - Derivative constant for the turn controller
     * @param turnIZone - Integral Zone constant for the turn controller
     * @param driveWheelDiam - Size of this wheel in mm
     */
    public SwerveModuleConstants(int idDriveMotor, boolean driveIsInverted, int idTurnMotor, boolean turnSensorPhase, boolean turnIsInverted, int turnMaxAllowedError, double turnP, double turnI, double turnD, int turnIZone, double driveWheelDiam) {
        //turn
        this.idTurnMotor = idTurnMotor;
        this.turnSensorPhase = turnSensorPhase;
        this.turnIsInverted = turnIsInverted;
        this.turnMaxAllowedError = turnMaxAllowedError;
        this.turnP = turnP;
        this.turnI = turnI;
        this.turnD = turnD;
        this.turnIZone = turnIZone;
        //drive
        this.idDriveMotor = idDriveMotor;
        this.driveIsInverted = driveIsInverted;
        this.driveWheelDiam = driveWheelDiam;
    }
}
