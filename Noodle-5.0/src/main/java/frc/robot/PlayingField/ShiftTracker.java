package frc.robot.PlayingField;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utility functions for keeping track of the state of a match.
 * In particular, we use the functions in this class to determine
 * who won auto, the current shift, time until we should start scoring, etc.
 */
public class ShiftTracker {

    private static final Timer teleopTimer = new Timer();

    public static final double teleopLengthSeconds = 140;
    public static final double autoLengthSeconds = 20;
    public static final double fullMatchLengthSeconds = teleopLengthSeconds + autoLengthSeconds;

    public static void signalStartOfTeleop() {
        teleopTimer.restart();
    }

    public static double getSecondsIntoTeleop() {
        if (!DriverStation.isTeleopEnabled()) {
            return -1;
        }
        return teleopTimer.get();
    }

    public static Optional<Shift> getCurrentShift() {
        // Note that AdvantageKit automatically grabs a single timestamp that's used by
        // all timers each robot loop. This makes sure the passage of time within
        // a robot loop doesn't cause different parts of the code to believe we are at
        // different stages within the match. We want each iteration of the main loop
        // to act as a snapshot of an instant in time. We don't want two robot mechanisms
        // following two different gameplans.
        for (Shift shift : Shift.values()) {
            if (shift.isHappeningNow()) {
                Logger.recordOutput("ShiftTracker/currentShift", shift.name());
                return Optional.of(shift);
            }
        }
        Logger.recordOutput("ShiftTracker/currentShift", "UNKNOWN");
        return Optional.empty();
    }

    public static Optional<Alliance> getWinnerOfAuto() {
        // See: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        String allianceThatWonAuto = DriverStation.getGameSpecificMessage();
        if (!allianceThatWonAuto.isEmpty() && allianceThatWonAuto.charAt(0) == 'R') {
            Logger.recordOutput("ShiftTracker/winnerOfAuto", "Red");
            return Optional.of(Alliance.Red);
        }
        if (!allianceThatWonAuto.isEmpty() && allianceThatWonAuto.charAt(0) == 'B') {
            Logger.recordOutput("ShiftTracker/winnerOfAuto", "Blue");
            return Optional.of(Alliance.Blue);
        }
        Logger.recordOutput("ShiftTracker/winnerOfAuto", "UNKNOWN");
        return Optional.empty();
    }

    public static Optional<Boolean> weWonAuto() {
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        Optional<Alliance> winnerOfAuto = ShiftTracker.getWinnerOfAuto();

        // Errors may occur to comms issues, or simply the fact that
        // auto hasn't finished yet, so a winner hasn't been decided!
        if (ourAlliance.isEmpty() || winnerOfAuto.isEmpty()) {
            Logger.recordOutput("ShiftTracker/weWonAuto", "UNKNOWN");
            return Optional.empty();
        }

        boolean weWonAuto = ourAlliance.get() == winnerOfAuto.get();
        Logger.recordOutput("ShiftTracker/weWonAuto", String.valueOf(weWonAuto));

        return Optional.of(weWonAuto);
    }

    public static boolean isOurTurnToScore() {
        // See: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        // See also: https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/567

        // Gather some information
        Optional<Shift> currentShift = ShiftTracker.getCurrentShift();
        Optional<Boolean> weWonAuto = ShiftTracker.weWonAuto();

        // If we can't tell what shift we're in, or whether or not we won auto, it's likely because
        // we're actually in auto right now, in which case we're allowed to score!
        // 
        // These optionals could also be empty if we're having comms issues with the field,
        // but I don't think it hurts to default to true in this case, so I'll leave it as is for now.
        if (currentShift.isEmpty() || weWonAuto.isEmpty()) {
            Logger.recordOutput("ShiftTracker/isOurTurnToScore", true);
            return true;
        }

        // We can always score during the TRANSITION_SHIFT and END_GAME
        if (currentShift.get() == Shift.TRANSITION_SHIFT || currentShift.get() == Shift.END_GAME) {
            Logger.recordOutput("ShiftTracker/isOurTurnToScore", true);
            return true;
        }

        // Whether or not we can score in other shifts depends on if we won auto.
        // The alliance that won auto gets to score in shifts 2 and 4,
        // while the alliance that lost auto gets to score in shifts 1 and 3.
        boolean isOurTurn = weWonAuto.get() && (currentShift.get() == Shift.SHIFT_2 || currentShift.get() == Shift.SHIFT_4);
        isOurTurn |= (!weWonAuto.get()) && (currentShift.get() == Shift.SHIFT_1 || currentShift.get() == Shift.SHIFT_3);
        
        Logger.recordOutput("ShiftTracker/isOurTurnToScore", isOurTurn);
        return isOurTurn;
    }

    // /** Returns whether or not the hub is active */
    // private static boolean wpiLibReferenceImplementation() {
    //     Optional<Alliance> alliance = DriverStation.getAlliance();
    //     // If we have no alliance, we cannot be enabled, therefore no hub.
    //     if (alliance.isEmpty()) {
    //         return false;
    //     }
    //     // Hub is always enabled in autonomous.
    //     if (DriverStation.isAutonomousEnabled()) {
    //         return true;
    //     }
    //     // At this point, if we're not teleop enabled, there is no hub.
    //     if (!DriverStation.isTeleopEnabled()) {
    //         return false;
    //     }

    //     // We're teleop enabled, compute.
    //     double matchTime = DriverStation.getMatchTime();
    //     String gameData = DriverStation.getGameSpecificMessage();
    //     // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    //     if (gameData.isEmpty()) {
    //         return true;
    //     }
    //     boolean redInactiveFirst = false;
    //     switch (gameData.charAt(0)) {
    //         case 'R' -> redInactiveFirst = true;
    //         case 'B' -> redInactiveFirst = false;
    //         default -> {
    //         // If we have invalid game data, assume hub is active.
    //         return true;
    //         }
    //     }

    //     // Shift was is active for blue if red won auto, or red if blue won auto.
    //     boolean shift1Active = switch (alliance.get()) {
    //         case Red -> !redInactiveFirst;
    //         case Blue -> redInactiveFirst;
    //     };

    //     if (matchTime > 130) {
    //         // Transition shift, hub is active.
    //         return true;
    //     } else if (matchTime > 105) {
    //         // Shift 1
    //         return shift1Active;
    //     } else if (matchTime > 80) {
    //         // Shift 2
    //         return !shift1Active;
    //     } else if (matchTime > 55) {
    //         // Shift 3
    //         return shift1Active;
    //     } else if (matchTime > 30) {
    //         // Shift 4
    //         return !shift1Active;
    //     } else {
    //         // End game, hub always active.
    //         return true;
    //     }
    // }
}
