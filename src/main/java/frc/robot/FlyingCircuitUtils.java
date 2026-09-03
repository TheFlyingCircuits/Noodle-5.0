package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlyingCircuitUtils {

    public static <T> T getAllianceDependentValue(T valueOnRed, T valueOnBlue, T valueWhenNoComms) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return valueOnRed;
            }
            if (alliance.get() == Alliance.Blue) {
                return valueOnBlue;
            }
        }

        // Should never get to this point as long as we're connected to the driver station.
        return valueWhenNoComms;
    }

    public static double getNumberFromDashboard(String name, double defaultValue) {
        // get the value from the dashboard, then echo it back to confirm
        // the value is being read correctly.
        double value = SmartDashboard.getNumber(name, defaultValue);
        SmartDashboard.putNumber(name, value);

        return value;
    }

    public static boolean getBooleanFromDashboard(String name, boolean defaultValue) {
        // get the value from the dashboard, then echo it back to confirm
        // the value is being read correctly.
        boolean value = SmartDashboard.getBoolean(name, defaultValue);
        SmartDashboard.putBoolean(name, value);

        return value;
    }

    public static void putNumberOnDashboard(String name, double numberToPutOnDashboard) {
        // This function honestly isn't really necessary. It's mostly here to have the
        // "symmetry" of using "FlyingCircuitUtils" to for reading/writing numbers from/to the dashboard,
        // instead of using "FlyingCircuitUtils" for reading, while using "SmartDashboard" for writing.
        // 
        // In practice, the dashboard is only really used for brief periods of time as we test and tune things,
        // and Logger.recordOutput() is the more appropriate tool for "printing" debug info.
        SmartDashboard.putNumber(name, numberToPutOnDashboard);
    }

    /** Intended for wrapping angles to a restricted range, while preserving the "direction" the angle points in.
     *  Can also be used for other purposes, like wrapping the position of an LED pattern back to the begining of
     *  an LED strip once it runs off the end. See comments within this function for more details.
     *  TODO: Better docs?
     */
    public static double circularClamp(double wrapMe, double inclusiveMin, double exclusiveMax) {
        // The easiest way to implement this function (in terms of understanding) is probably:
        //   double distanceBetweenEquivalentValues =  exclusiveMax - inclusiveMin;
        //   while (wrapMe <  min) { wrapMe += distanceBetweenEquivalentValues; }
        //   while (wrapMe >= max) { wrapMe -= distanceBetweenEquivalentValues; }
        //   return wrapMe;
        //
        // However, you hopefully get the feeling that there's a way to just calculate the answer
        // directly instead of looping until you're in the desired range, which is what we do below.

        // 1) Find the number that we're allowed to add or subtract from "wrapMe" without
        //    changing its effective value.
        //    (e.g. when wrapping angles measured in degrees, this value is 360. Adding or subtracting
        //     360 to any given angle results in an angle pointing in the same direction as the original.)
        double distanceBetweenEquivalentValues =  exclusiveMax - inclusiveMin;

        // 2) Find the "t" value for if "wrapMe" had been obtained by
        //    linearly interpolating between the min and the max.
        double t = (wrapMe - inclusiveMin) / distanceBetweenEquivalentValues;

        // 3) Use the non-fractional part of "t" (i.e. the part that's before the decimal place) to calculate the "index"
        //    of the current range. For example, if we're clamping to the range 0 to 360, then:
        //
        //      [0-360) has index 0
        //      [360-720) has index 1
        //      [720-1080) has index 2
        //      [-360-0) has index -1
        //      [-720-360) has index -2, etc.
        double indexOfCurrentRange = Math.floor(t);
        // Note: You need to use Math.floor() here as opposed to just casting to int. This is because casting
        //       to int always rounds towards 0, whereas the floor of a negative number rounds away from 0.
        //       See: https://www.desmos.com/calculator/pnmjpb1whn

        // 4) Get the final result by subtracting the appropriate amount of
        //    full ranges from the given number.
        return wrapMe - (distanceBetweenEquivalentValues * indexOfCurrentRange);
    }

    /** Positive to the left of the line, negative to the right of the line */
    public static double signedDistanceToLine(Translation2d point, Pose2d line) {
        Translation2d anchor = line.getTranslation();
        Translation2d anchorToPoint = point.minus(anchor);

        // normalToLine = new Translation2d(-line.getRotation().getSin(), line.getRotation().getCos());
        double normalToLineX = -line.getRotation().getSin();
        double normalToLineY =  line.getRotation().getCos();

        // project onto line normal to get signed distance (same as <directionVectorAlongLine> cross <anchorToPoint>)
        return anchorToPoint.getX() * normalToLineX + anchorToPoint.getY() * normalToLineY;

        // interesting alternative implementation:
        // Pose2d pointAsPose = new Pose2d(point, Rotation2d.kZero);
        // return pointAsPose.relativeTo(line).getY();
    }


    // Useful Unicode Symbols for ASCII art
    // ↑ ← → ↓
    // ↖ ↗ ↘ ↙
}
