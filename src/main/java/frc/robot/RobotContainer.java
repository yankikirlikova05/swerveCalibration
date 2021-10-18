// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.lib.util.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ShooterPID;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Swerve swerveDrivetrain = new Swerve(false);
  Joystick driver = new Joystick(0);
  SwerveDriveCommand driveCommand = new SwerveDriveCommand(swerveDrivetrain, driver);
  LEDSubsystem LED = new LEDSubsystem();

  
  public Shooter shooter = new Shooter();
  public ShooterPID shooterpid = new ShooterPID(shooter);
  AutoShoot autoShoot = new AutoShoot(LED, shooter, swerveDrivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // commented out because shooter is broken
    // new JoystickButton(driver, 3).whileHeld(shooterpid);

    swerveDrivetrain.setDefaultCommand(driveCommand);
    JoystickButton ledButton = new JoystickButton(driver, 1);

    ledButton.whenPressed(new RunCommand(()-> LED.turnOn(), LED));
    ledButton.whenReleased(
      new RunCommand(
        () -> LED.turnOff(),
        LED)
    );
    new JoystickButton(driver,2).whenHeld(autoShoot);
   
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
