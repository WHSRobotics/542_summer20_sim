package util;

public class SwerveConstants {

    public static double MAX_ACCELERATION = 800.0; // mm/s^2
    public static double MAX_VELOCITY = 1400.0;
    public static double kP = 0.0001;
    public static double kV = 0.0006338;
    public static double kA = 0.0012;
    public static double lookaheadDistance = 350;//220;
    public static double velocityConstant = 2.0;
    public static double MAX_ANGULAR_ACCELERATION = 25;//5.2;
    public static double MAX_ANGULAR_VELOCITY = 108;
    public static double hKP = 10;
    public static double hKI = 0.0;
    public static double hKD = 0.0;

    public static class StartToFoundationSwerveConstants {
        public static double kP = util.SwerveConstants.kP/2.54;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class FoundationToWallSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class WallToSkystoneSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class StartToSkystoneSwerveConstants {
        public static double kP = util.SwerveConstants.kP/2;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant*1.1;
    }

    public static class SkystoneToMovedFoundationSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class SkystoneToUnmovedFoundationSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance + 50;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class MovedFoundationToParkingSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance + 200;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }

    public static class WallToParkingSwerveConstants {
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }
    public static class secondSkystoneToMovedFoundationSwerveConstants{
        public static double kP = util.SwerveConstants.kP;
        public static double kV = util.SwerveConstants.kV;
        public static double kA = util.SwerveConstants.kA;

        public static double lookaheadDistance = util.SwerveConstants.lookaheadDistance;
        public static double velocityConstant = util.SwerveConstants.velocityConstant;
    }





}
