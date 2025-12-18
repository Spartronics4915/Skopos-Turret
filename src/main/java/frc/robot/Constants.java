// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    
    public static final class ShooterConstants {
    
        public static final int SHOOTER_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum ShooterState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private ShooterState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class IntakeConstants {
    
        public static final int INTAKE_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum IntakeState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private IntakeState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class HoodConstants {
        
        public static final int HOOD_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum HoodState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private HoodState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class TurretConstants {
        
        public static final int TURRET_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum TurretState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private TurretState(Rotation2d angle) {
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
