package frc.team1918.robot;

import edu.wpi.first.math.MathUtil;
import frc.team1918.robot.constants.*; 


public class Helpers {
    //Helpers for Debugging
    public static final class Debug {
        private static boolean debugEnabled = GlobalConstants.DEBUG_ENABLED_DEFAULT;
        /**
         * This function takes a string and outputs it to the console when the debugging is enabled
         * @param message String to print to console
         */
        public final static void debug(String message) {
            if (debugEnabled) {
                System.out.println(message);
            }
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
    //OI Helpers
    public static final class OI {
    }

    //General Helpers
    public static final class General {
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
            return (ticks * Math.PI / (SwerveConstants.kTurnEncoderFullRotation / 2));
        }

        /**
         * This function takes radians and returns encoder ticks (based on a 0 offset)
         * @param rads double precision value in radians
         * @return integer value in encoder ticks
         */
        public final static int radiansToTicks(double rads) {

			return (int) (rads / Math.PI * (SwerveConstants.kTurnEncoderFullRotation / 2));
        }
    }
}