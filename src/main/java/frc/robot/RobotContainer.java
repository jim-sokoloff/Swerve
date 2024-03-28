// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetLEDs;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RGB.LEDColor;
import frc.robot.subsystems.RGB.LEDMode;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  public static final CommandXboxController driver = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;
  // /* Driver Buttons */
  // public static final Trigger zeroGyro = driver.button(2);
  // private final Trigger robotCentric = driver.button(3);
  // private final Trigger liftSolenoid = driver.button(6);
  // private final Trigger blockSolenoid = driver.button(1);
  // private final Trigger grabNote = driver.button(7);
  // public static final Trigger launchNote = driver.button(8); 

  /* Subsystems */
  private final Swerve m_swerve = new Swerve();
  //private final Pneumatics m_pneumatics = new Pneumatics();
  //private final Launcher m_launcher = new Launcher();
  private final RGB m_RGB = new RGB(0);

  /* Alliance */
  private static boolean m_allianceIsBlue = false;
  private static int m_allianceStationNumber = 7;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // System.out.println(translationAxis);
    // System.out.println(strafeAxis);
    // System.out.println(rotationAxis);
    //m_pneumatics.reverseLiftSolenoid();
    //m_pneumatics.reverseBlockSolenoid();
    m_swerve.setDefaultCommand(createSwerveCommand(true));
    // Moving the RGB to use commands so that you can control the LED states here, rather than having to add more inputs directly inside the RGB class. This allows you to tack a SetLeds command onto triggers in this class directly
    m_RGB.setDefaultCommand(new RunCommand(() -> m_RGB.setLedState(m_allianceIsBlue ? LEDColor.Blue : LEDColor.Red, LEDMode.Loop), m_RGB));

    // Configure the button bindings
    configureButtonBindings();
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    // SmartDashboard.putNumber("rotationAxis", rotationAxis);

  }

  private TeleopSwerve createSwerveCommand(boolean isRobotCentric) {
    return new TeleopSwerve(
      m_swerve,
      () -> -driver.getLeftY(),
      () -> -driver.getLeftX(),
      () -> -driver.getRightX() / 3,
      () -> isRobotCentric
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // switches on controller labled from left to right: leftmost is number one, next is number two when it is closest to the joystick, and 3 when furthest, next is 4 when closest to joystick, 5 when furthest, last is 6 when closest. 
    // left button on bottom is 7, right button is 8
    // Power buttons, dials don't seem to do anything
    //left button pickup
    // right button launch
    // far right stick up-- activate hooks
    // right stick down-- hooks down
    // left stick-- blockers up blockers down
    // 5 -> inverse 2
    // 4 -> 3
    // 1 -> inverse 1
    // 0 -> 0
    // Rather than holding down a button to enable robot centric mode, I recommend a toggle system.
    // The WPILib command and subsytem requirements will handle this well for you
    driver.back().onTrue(createSwerveCommand(true));
    driver.start().onTrue(createSwerveCommand(false));

    // Sample launch button - StartEndCommand is helpful for this
    // driver.rightTrigger().whileTrue(new StartEndCommand(() -> m_launcher.setLaunchMotors(), () -> m_launcher.stopLaunchMotors(), m_launcher).alongWith(new SetLEDs(m_RGB, LEDColor.Green, LEDMode.Solid)));

    // A simple inline function to create a function that resets the odometry angle, but then keeps the SetLEDs command running until the command is ended/interupted
    Function<Rotation2d, Command> getResetHeadingCommand = (Rotation2d rotation) -> 
      new InstantCommand(() -> m_swerve.resetHeading(Rotation2d.fromDegrees(0)))
      .andThen(new SetLEDs(m_RGB, LEDColor.Gray, LEDMode.Solid));

    // I put this on the d-pad because 131 uses the 4 main directions to fix our orientaion (if the robot is facing left, we'll tell the gyro we're at 90*)
    // This only matters once you've gotten field centric working
    driver.povUp().whileTrue(getResetHeadingCommand.apply(Rotation2d.fromDegrees(0)));
    driver.povLeft().whileTrue(getResetHeadingCommand.apply(Rotation2d.fromDegrees(90)));
    driver.povDown().whileTrue(getResetHeadingCommand.apply(Rotation2d.fromDegrees(180)));
    driver.povRight().whileTrue(getResetHeadingCommand.apply(Rotation2d.fromDegrees(-90)));

    /* // Replace these with StartEndCommands (maybe set the requirements on them?)
    liftSolenoid.onTrue(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    liftSolenoid.onFalse(new InstantCommand(() -> m_pneumatics.toggleLiftSolenoid()));
    blockSolenoid.onTrue(new InstantCommand(() -> m_pneumatics.toggleBlockSolenoid()));
    blockSolenoid.onFalse(new InstantCommand(() -> m_pneumatics.toggleBlockSolenoid()));
    grabNote.onTrue(new InstantCommand(() -> m_launcher.setGrabMotors()));
    grabNote.onFalse(new InstantCommand(() -> m_launcher.stopGrabMotors()));
    launchNote.onTrue(new InstantCommand(() -> m_launcher.setLaunchMotors()));
    launchNote.onFalse(new InstantCommand(() -> m_launcher.stopLaunchMotors()));
    */

    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        2.0));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
        new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        new PathConstraints(
            4.0, 4.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0,
        0));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {

      Pose2d currentPose = m_swerve.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }


  public static boolean AllianceIsBlue() {
    return m_allianceIsBlue;
  }

  public static boolean AllianceIsRed() {
    return !AllianceIsBlue();
  }

  public static int AllianceStationNumber() {
    return m_allianceStationNumber;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }

  public final void UpdateAlliance() {
    DriverStation.Alliance color;
	  color = DriverStation.getAlliance().orElse(Alliance.Blue);
	  if (color == DriverStation.Alliance.Blue) {
      m_allianceIsBlue = true;
    } else {
      m_allianceIsBlue = false;
    }

    // 5 is an error of sorts.
    m_allianceStationNumber = DriverStation.getLocation().orElse(5);
  }

}

