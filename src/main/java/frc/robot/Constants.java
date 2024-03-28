package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.config.SwerveModuleConstants;
import static edu.wpi.first.units.Units.*;

public final class Constants {
  // Debugging swerve helper
  public final static boolean JTS_false = false;
  public final static boolean JTS_true = true;
  public final static boolean JTS_useCAN = true;

  public static final class Swerve {

    public static final double stickDeadband = 0.1;
    public static final double stickDeadbandHigh = 0.5;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final Measure<Distance> trackWidth = Inches.of(25.75);
    public static final Measure<Distance> wheelBase = Inches.of(21.4);
    public static final Measure<Distance> wheelDiameter = Meters.of(Meters.convertFrom(4.0, Inches));
    public static final Measure<Distance> wheelCircumference = Meters.of(wheelDiameter.magnitude() * Math.PI);

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    // original: public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807
    public static double mk4iL1DriveGearRatio = 6.75;

    public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

    public static double driveGearRatio = mk4iL1DriveGearRatio;

    public static double angleGearRatio = mk4iL1TurnGearRatio;

    public static final Translation2d flModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
        trackWidth.magnitude() / 2.0);
    public static final Translation2d frModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
        -trackWidth.magnitude() / 2.0);
    public static final Translation2d blModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
        trackWidth.magnitude() / 2.0);
    public static final Translation2d brModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
        -trackWidth.magnitude() / 2.0);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30;   // was 20
    public static final int driveContinuousCurrentLimit = 50;   // was 30

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.455; // meters per second
    public static final double maxAngularVelocity = 1.5;

    public static final double maxSpeedLow = 1.25; // meters per second
    public static final double maxAngularVelocityLow = 0.5;

    /* Angle Motor PID Values */
    // Tested with robot tilted, no weight on wheels, 0 I and 0 D
    // Vibrations at 0.025 (minimal), 0.015 (almost undetectable), 0.010 (rare buzz), 0.005 (rare, light buzz)
    // Then P 0.01 (heavy unstable), 0.0001 (stable), 0.001 (unstable), 0.0002 (okay, but flippy)
    public static final double angleKP = 0.030;
    public static final double angleKI = 0.0000;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.00;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = .05;// 90% feed forward

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 3.04;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kCoast;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    public static String[] modNames = { "FL ", "FR ", "BL ", "BR " };

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 9;
      public static final int cancoderID = 20;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 2;
      public static final int cancoderID = 21;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90);     // Zeroed in Phoenix Tuner X
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 12;
      public static final int cancoderID = 22;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270);     // Zeroed in Phoenix Tuner X
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 4;
      public static final int cancoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          cancoderID, angleOffset);
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0, 0), // Translation constants
        new PIDConstants(5.0, 0, 0), // Rotation constants
        maxSpeed,
        flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
        new ReplanningConfig());
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5; //original value: 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //original value: 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.05;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
