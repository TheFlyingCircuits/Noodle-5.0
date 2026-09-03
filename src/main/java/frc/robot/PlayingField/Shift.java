package frc.robot.PlayingField;

public enum Shift {
    /* From Game Manual Section 6.4
     * Auto Length: 20 seconds
     * Teleop Length (including shifts and endgame): 2 minutes and 20 seconds <===> 140 total seconds
     * Total Match Length: 2 minutes and 40 seconds <===> 160 total seconds
     * 
     * Part Of Match | Duration (seconds) | Match Clock (seconds)
     * --------------+--------------------+-----------------------
     *      AUTO     |         20         |       20 -> 0
     * --------------+--------------------+-----------------------
     *   Transition  |         10         |      140 -> 130
     * --------------+--------------------+-----------------------
     *     Shift 1   |         25         |      130 -> 105
     * --------------+--------------------+-----------------------
     *     Shift 2   |         25         |      105 -> 80
     * --------------+--------------------+-----------------------
     *     Shift 3   |         25         |       80 -> 55
     * --------------+--------------------+-----------------------
     *     Shift 4   |         25         |       55 -> 30
     * --------------+--------------------+-----------------------
     *    End Game   |         30         |       30 -> 0
     * --------------+--------------------+-----------------------
    */
    // For our purposes, I think it will be cleaner to just treat auto seperately
    // than the rest of the shifts in our shooting logic.
    TRANSITION_SHIFT(0, 10),
    SHIFT_1(10, 35), SHIFT_2(35, 60),
    SHIFT_3(60, 85), SHIFT_4(85, 110),
    END_GAME(110, 140);

    /** The amount of seconds between the start of teleop and the start of this shift. */
    private final double startTimestamp;
    /** The amount of seconds between the start of teleop and the end of this shift. */
    private final double endTimestamp;

    private Shift(double start, double end) {
        this.startTimestamp = start;
        this.endTimestamp = end;
    }

    /** 
     * Strictly speaking, this function (and all other timing related functions in this class) can
     * only give us an ESTIMATE of the shift's status, because the official match time isn't sent
     * to the robots from FMS, and our local timers may drift relative to the offical timer that's
     * controlling the field. However, I'd be surprised if the drift was any more than a second or two
     * over the course of the full 2:40 match, so we're going to assume / cross our fingers that any drift
     * is inconsequential until we have evidence otherwise.
     * 
     * See also: https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/567
     */
    public boolean isHappeningNow() {
        double secondsIntoTeleop = ShiftTracker.getSecondsIntoTeleop();
        return (this.startTimestamp <= secondsIntoTeleop) && (secondsIntoTeleop < this.endTimestamp);
    }

    /** This function tells you the amount of seconds between now and the start of the shift.
     *  If the shift hasn't started yet, then this value will be positive.
     *  If the shift has already started, then this value will be negative.
     */
    public double getSecondsTillStart() {
        return this.startTimestamp - ShiftTracker.getSecondsIntoTeleop();
    }

    /** This function tells you the amount of seconds between now and the end of the shift.
     * 
     *  If the shift hasn't ended yet, then this value will be positive. Keep in mind that one reason
     *  the shift may not have ended yet is that the shift hasn't even started yet!
     * 
     *  If the shift has already ended, then this value will be negative.
     */
    public double getSecondsTillEnd() {
        return this.endTimestamp - ShiftTracker.getSecondsIntoTeleop();
    }

    /** This function tells you the amount of seconds between now and the start of the shift.
     *  If the shift hasn't started yet, then this value will be negative.
     *  If the shift has already started, then this value will be positive.
     *  (and will continue to be positive even after the shift ends!)
     */
    public double getSecondsSinceStart() {
        return ShiftTracker.getSecondsIntoTeleop() - this.startTimestamp;
    }

    /** This function tells you the amount of seconds between now and the end of the shift.
     * 
     *  If the shift has already ended, then this value will be positive.
     * 
     *  If the shift hasn't ended yet, then this value will be negative. Keep in mind that one reason
     *  the shift may not have ended yet is that the shift hasn't even started yet!
     */
    public double getSecondsSinceEnd() {
        return ShiftTracker.getSecondsIntoTeleop() - this.endTimestamp;
    }
}
