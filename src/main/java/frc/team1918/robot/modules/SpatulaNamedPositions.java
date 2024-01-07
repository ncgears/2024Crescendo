package frc.team1918.robot.modules;

public class SpatulaNamedPositions {
    //control
    public final double home, griddle, clear, floor, bump, limit, store, midUp, midDown;

    /**
     * Creates a new positions object for storing named positions
     * @param MotorID - (int) CAN ID of the Motor (TalonSRX)
     * @param SensorPhase - (bool) true to invert the sensor phase
     * @param SensorTicks - (int) count of encoder ticks in a full rotation
     * @param SensorNotContinuous - (boolean) set sensorNotContinuous (for things with less than full rotation)
     * @param IsInverted - (bool) true to invert the motor control
     * @param AllowedError - (int) maximum allowable error
     */
    public SpatulaNamedPositions(double home, double griddle, double clear, double floor, double bump, double limit, double store, double midUp, double midDown) {
        this.home = home;
        this.griddle = griddle;
        this.clear = clear;
        this.floor = floor;
        this.bump = bump;
        this.limit = limit;
        this.store = store;
        this.midUp = midUp;
        this.midDown = midDown;
    }
}
