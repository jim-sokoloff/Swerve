package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int m_driveMotorID;
  public final int m_angleMotorID;
  public final int m_canCoderID;
  public final Rotation2d m_angleOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset) {
    m_driveMotorID = driveMotorID;
    m_angleMotorID = angleMotorID;
    m_canCoderID = cancoderID;
    m_angleOffset = angleOffset;
  }
}
