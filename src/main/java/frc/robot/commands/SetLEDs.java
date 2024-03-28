// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RGB;
import frc.robot.subsystems.RGB.LEDColor;
import frc.robot.subsystems.RGB.LEDMode;

public class SetLEDs extends Command {

  private final RGB m_rgb;
  private final LEDColor m_color;
  private final LEDMode m_mode;

  /** Creates a new SetLEDs. */
  public SetLEDs(RGB rgb, LEDColor color, LEDMode mode) {
    m_rgb = rgb;
    m_color = color;
    m_mode = mode;
    addRequirements(rgb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // I think you only need to call this in `initalize` and not `execute`
    m_rgb.setLedState(m_color, m_mode);
  }
}
