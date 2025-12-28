package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {

    public static final class HoodConstants {
        
        public static final int HOOD_MOTOR_ID = 15;
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(350);
        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        
        public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = motorConfiguration.Slot0;
                slot0Configs.kS = 0.25;
                slot0Configs.kV = 0.12;
                slot0Configs.kA = 0.01;
                slot0Configs.kP = 50;
                slot0Configs.kI = 0.05;
                slot0Configs.kD = 0.15;

            MotionMagicConfigs motionMagicConfigs = motorConfiguration.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = 3.5;
                motionMagicConfigs.MotionMagicExpo_kV = 0.15;
                motionMagicConfigs.MotionMagicExpo_kA = 0.1;
            
            FeedbackConfigs feedbackConfigs = motorConfiguration.Feedback;
                feedbackConfigs.SensorToMechanismRatio = 2.4936523444;
        }

        public enum HoodState {
            PLACE_HOLDER(Rotation2d.fromDegrees(90));

            public Rotation2d angle;

            private HoodState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = 22.475 / 12;
        public static final double WHEEL_BASE = 22.475 / 12;
        public static final double CHASSIS_RADIUS = Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);

        public static final double MAX_SPEED = Units.feetToMeters(24);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_SPEED * Math.PI / CHASSIS_RADIUS);

        public static boolean IS_FIELD_RELATIVE = false;

        public static final double STICK_DEADBAND = 0.1;

        public static Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);

        public static final int DRIVE_CONTROLLER_PORT = 0;
    }
}   
