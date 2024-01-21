package frc.team1918.robot;

public class Helpers {
    //Helpers for Debugging
    public static final class Debug {
        private static boolean debugEnabled = Constants.Global.DEBUG_ENABLED_DEFAULT;
        /**
         * This function takes a string and outputs it to the console when the debugging is enabled
         * @param message String to print to console
         */
        public final static void debug(String message) {
            if (debugEnabled) {
                System.out.println(message);
            }
        }
        public final static int debug(String message, int ticks) {
            if (debugEnabled) {
                if(debugThrottleMet(ticks)) System.out.println(message);
            }
            return ticks++;
        }
        public final static boolean debugThrottleMet(int ticks) {
            return (ticks % Constants.Global.DEBUG_RECURRING_TICKS == 0);
        }

        /**
         * This function toggles the debugging output to console. In a future version, each press will increase the debug level
         * through a set list of severity levels.
         */
        public final static void toggleDebug() {
            debugEnabled = !debugEnabled;
            System.out.println("Debugging Output=" + debugEnabled);
        }
    }
    //General Helpers
    public static final class General {
        /**
         * This function takes values a and b. It determines the minimum delta between the two based on the wrap value.
         * @param a 
         * @param b
         * @param wrap
         * @return
         */
        public final static double minChange(double a, double b, double wrap) {
            return Math.IEEEremainder(a - b, wrap);
        }
        public final static int minChange(int a, int b, int wrap) {
            return (int) Math.IEEEremainder(a - b, wrap);
        }
        
        public static double encoderToMeters(double encoder, double wheelDiam) {
            return (double) encoder * (Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR * wheelDiam * Math.PI) / Constants.DriveTrain.DT_DRIVE_ENCODER_FULL_ROTATION / 1000.0;
        }
        public static double metersToEncoder(double meters, double wheelDiam) {
            return (double) meters * 1000.0 * Constants.DriveTrain.DT_DRIVE_ENCODER_FULL_ROTATION / (Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR * wheelDiam * Math.PI);
        }
        
        public final static double roundDouble(double val, int decimals) {
            return Math.round(val * Math.pow(10,decimals)) / Math.pow(10,decimals);
            // final DecimalFormat df = new DecimalFormat(format);
            // return df.format(val);
        }

        /**
         * This function converts encoder ticket and returns the value in radians
         * @param ticks (integer) value in encoder ticks
         * @return (double) value in radians
         */
        public final static double ticksToRadians(int ticks) {
            return (ticks * Math.PI / (Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION / 2));
        }

        /**
         * This function takes radians and returns encoder ticks (based on a 0 offset)
         * @param rads double precision value in radians
         * @return integer value in encoder ticks
         */
        public final static int radiansToTicks(double rads) {

			return (int) (rads / Math.PI * (Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION / 2));
        }

        /**
         * This function takes a value and modifies it by a gear reduction (or multiplication) for a single reduction
         * @param value double precision value of input
         * @param gearOne integer value of the number of teeth on the input side of the gear set (motor side)
         * @param gearTwo integer value of the number of teeth on the output side of the gear set (wheel side)
         * @return double precision value after gear reduction (or multiplication)
         */
        public final static double gearCalcSingle(double value, int gearOne, int gearTwo) {
            return value * (gearOne / gearTwo);
        }
        /**
         * This function takes a value and modifies it by a gear reduction (or multiplication) for a single reduction
         * @param value double precision value of input
         * @param firstGearOne integer value of the number of teeth on the input side of the first gear set (motor side)
         * @param firstGearTwo integer value of the number of teeth on the output side of the first gear set (wheel side)
         * @param secondGearOne integer value of the number of teeth on the output side of the second gear set (wheel side)
         * @param secondGearTwo integer value of the number of teeth on the output side of the second gear set (wheel side)
         * @return double precision value after gear reduction (or multiplication)
         */
        public final static double gearCalcDouble(double value, int firstGearOne, int firstGearTwo, int secondGearOne, int secondGearTwo) {
            return value * (firstGearOne / firstGearTwo) * (secondGearOne / secondGearTwo);
        }

        /**
         * This function takes a value in rotations per second and converts it to meters per second based on the size of the wheel
         * @param rps (double) Rotations Per Second
         * @param wheelDiamMM (double) Diameter of the wheel in mm
         * @return (double) meters per second based on the wheel size and rotations per second
         */
        public final static double rpsToMetersPerSecond(double rps, double wheelDiamMM) {
            return ((rps) * (wheelDiamMM * Math.PI)) / 1000;
        }
        /**
         * This function takes a value in a speed in meters per second and converts to rotations per second based on the size of the wheel
         * @param mps (double) Meters Per Second
         * @param wheelDiamMM (double) Diameter of the wheel in mm
         * @return (double) rotations per second based on wheel size and speed
         */
        public final static double metersPerSecondToRPS(double mps, double wheelDiamMM) {
            return ((mps * 1000) / (wheelDiamMM * Math.PI));
        }

        /**
         * Converts RPS to Ticks per 100ms
         * @param rps - RPS to convert
         * @param fullRotationTicks - Number of ticks in a full rotation (encoder dependent)
         * @return
         */
        public final static double rpsToTicksPer100ms(double rps, int fullRotationTicks, double factor) {
            factor = 0.58;
            return ((rps * fullRotationTicks) / 10 / factor);
        }

        /**
         * Converts Ticks per 100ms to RPS
         * @param ticks - Ticks per 100ms to convert
         * @param fullRotationTicks - Number of ticks in a full rotation (encoder dependent)
         * @param factor - Factor to multiple by (for getting rpm at wheel)
         * @return
         */
        public final static double ticksPer100msToRPS(double ticks, int fullRotationTicks, double factor) {
            return ((ticks / fullRotationTicks) * 10 * factor);
        }
    }
}