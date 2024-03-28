package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;

  private final CANcoder m_turnCancoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private SwerveModuleState currentDesiredState = new SwerveModuleState();

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.m_angleOffset;

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.m_angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    m_turnCancoder = new CANcoder(moduleConstants.m_canCoderID, "rio");

    // I actually recommend doing this in the code, not directly in Phoenix Tuner X. If you ever have to swap out a motor (we've had that at competitions many times), it makes replacement faster (all you need to do is set up the CAN ID).
    // Maybe that matters less for CanCoders since you'll need to measure a new absolute angle, but it's also nice to have the actual offsets stored in git
    var can_config = new CANcoderConfiguration();
    can_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    can_config.MagnetSensor.MagnetOffset = angleOffset.getRotations();
    m_turnCancoder.getConfigurator().apply(can_config);

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.m_driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    currentDesiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public SwerveModuleState getDesiredState() {
    return currentDesiredState;
  }

  public void resetAngleToAbsolute() {
    // If you set the absolute offsets so 0* on the CANcoder is facing directly forward on the robot, then this code can just return the exact value
    integratedAngleEncoder.setPosition(Rotation2d.fromRotations(m_turnCancoder.getAbsolutePosition().getValueAsDouble()).getDegrees());

  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);

    // ******************************************************** */

    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);

    // ******************************************************** */

    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    angleMotor.burnFlash();

  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kMinimal);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.driveKP);
    driveController.setI(Constants.Swerve.driveKI);
    driveController.setD(Constants.Swerve.driveKD);
    driveController.setFF(Constants.Swerve.driveKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    SmartDashboard.putBoolean("isOpenLoop - setSpeed", isOpenLoop);
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput); // You have voltage compensation enabled enabled, so I wonder if this is good enough?
      // driveMotor.setVoltage(percentOutput * RobotController.getBatteryVoltage());
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          CANSparkBase.ControlType.kVelocity,
          0);
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    // JTS - HACK: undo that for testing
    //angle = desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(Constants.Swerve.modNames[moduleNumber] + "cancoder",
        round2dp(m_turnCancoder.getAbsolutePosition().getValueAsDouble(), 2));
    SmartDashboard.putNumber(Constants.Swerve.modNames[moduleNumber] + "intcoder",
        round2dp(integratedAngleEncoder.getPosition(), 2));

    SmartDashboard.putNumber("CAN-2-angle" + Constants.Swerve.modNames[moduleNumber], m_turnCancoder.getAbsolutePosition().getValue());
    SmartDashboard.putString("CAN-2-ver" + Constants.Swerve.modNames[moduleNumber], m_turnCancoder.getVersion().toString());

  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    return Math.round(number * temp)/ temp;
  }

}
