package frc.utils;
//Use this as utils class for logging in robot code.
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;

public class Logging {
    private static final String RESET = "\u001B[0m";
    private static final String BLUE = "\u001B[34m";
    private static final String GREEN = "\u001B[32m";
    private static final String YELLOW = "\u001B[33m";
    private static final String RED = "\u001B[31m";
    private static final String PURPLE = "\u001B[35m";
    @Setter
    private static String printFormat = "[MT %matchStage% %matchTime%][RT %rioTime%][%level%][%tag%] %msg%\n";
    @Setter
    private static Level filterLevel = Level.DEBUG;

    private static boolean shouldLog(Level level) {
        return level.ordinal() >= filterLevel.ordinal();
    }

    private static String colorize(Level level, String text) {
        return switch (level) {
            case DEBUG -> BLUE + text + RESET;
            case INFO -> GREEN + text + RESET;
            case WARNING -> YELLOW + text + RESET;
            case ERROR -> RED + text + RESET;
            case CRITICAL -> PURPLE + text + RESET;
        };
    }

    public static void log(Level level, String tag, String format, Object... arguments) {
        if (!shouldLog(level)) return;

        String formattedMessage = String.format(format, arguments);
        String threadName = Thread.currentThread().getName();
        double rioTime = Timer.getFPGATimestamp();
        double matchTime = Timer.getMatchTime();
        String matchStage = DriverStation.isAutonomousEnabled() ? "Auto" : DriverStation.isTeleopEnabled() ? "Tele" : "Prep";

        String output = printFormat
                .replace("%matchStage%", matchStage)
                .replace("%matchTime%", String.format("%.4f", matchTime))
                .replace("%rioTime%", String.format("%.4f", rioTime))
                .replace("%level%", colorize(level, level.name()))
                .replace("%tag%", colorize(level, tag))
                .replace("%thread%", threadName)
                .replace("%msg%", formattedMessage);

        System.out.print(output);
    }

    // Convenience methods
    public static void debug(String tag, String format, Object... args) {
        log(Level.DEBUG, tag, format, args);
    }

    public static void info(String tag, String format, Object... args) {
        log(Level.INFO, tag, format, args);
    }

    public static void warn(String tag, String format, Object... args) {
        log(Level.WARNING, tag, format, args);
    }

    public static void error(String tag, String format, Object... args) {
        log(Level.ERROR, tag, format, args);
    }

    public static void critical(String tag, String format, Object... args) {
        log(Level.CRITICAL, tag, format, args);
    }

    public enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    }
}
