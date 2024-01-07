package frc.team1918.robot.utils;

public class TalonConstants {
    //control
    public final int MotorID;
    public final boolean SensorPhase;
    public final int SensorTicks; //Number of encoder ticks for the encoder
    public final boolean SensorNotContinuous; //usually, true if within 1 rotation, false if it wraps past 0
    public final boolean IsInverted;
    public final int AllowedError;

    /**
     * Creates a new constants object for initializing a TalonSRX Motor Controller
     * @param MotorID - (int) CAN ID of the Motor (TalonSRX)
     * @param SensorPhase - (bool) true to invert the sensor phase
     * @param SensorTicks - (int) count of encoder ticks in a full rotation
     * @param SensorNotContinuous - (boolean) set sensorNotContinuous (for things with less than full rotation)
     * @param IsInverted - (bool) true to invert the motor control
     * @param AllowedError - (int) maximum allowable error
     */
    public TalonConstants(int MotorID, boolean SensorPhase, int SensorTicks, boolean SensorNotContinuous, boolean IsInverted, int AllowedError) {
        this.MotorID = MotorID;
        this.SensorPhase = SensorPhase;
        this.SensorTicks = SensorTicks;
        this.SensorNotContinuous = SensorNotContinuous;
        this.IsInverted = IsInverted;
        this.AllowedError = AllowedError;
    }
}
