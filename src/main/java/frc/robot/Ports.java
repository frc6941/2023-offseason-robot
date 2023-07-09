package frc.robot;

public class Ports {
    public static class CanId {
        public static class Rio {
            public static final int DRIVETRAIN_FRONT_LEFT_DRIVE = 0;
            public static final int DRIVETRAIN_FRONT_LEFT_STEER = 1;
            public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE = 2;
            public static final int DRIVETRAIN_FRONT_RIGHT_STEER = 3;
            public static final int DRIVETRAIN_BACK_LEFT_DRIVE = 4;
            public static final int DRIVETRAIN_BACK_LEFT_STEER = 5;
            public static final int DRIVETRAIN_BACK_RIGHT_DRIVE = 6;
            public static final int DRIVETRAIN_BACK_RIGHT_STEER = 7;

            public static final int HOPPER = 18;
            public static final int INDICATOR = 19;
        }

        public static class Canivore {
            public static final String CANIVORE_NAME = "canivore";

            // V8
            public static final int HOOD = 8;
            public static final int INDEXER_TUNNEL = 9;
            public static final int INDEXER_EJECTOR = 10;
            public static final int CLIMBER_PUSHER = 11;
            public static final int CLIMBER_HOOK = 12;
            public static final int SHOOTER_LEAD = 13;
            public static final int SHOOTER_FOLLOW = 14;
            public static final int TRIGGER = 15;
            public static final int INTAKE_DEPLOY = 16;
            public static final int INTAKE_ROLLER = 17;
        }
    }

    public static class AnalogInputId {
        public static final int BOTTOM_BEAM_BREAK_CHANNEL = 0;
        public static final int TOP_BEAM_BREAK_CHANNEL = 1;
    }

    public static class Controller {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }
}
