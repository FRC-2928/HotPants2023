// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;

import frc.robot.commands.DriveCommand;
import frc.robot.oi.DriverOI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();

    private final XboxController controller = new XboxController(0);
    public final DriverOI driverOI = new DriverOI(this.controller);
    
    public RobotContainer() {
        drivetrain.register();

        drivetrain.setDefaultCommand(new DriveCommand(
                drivetrain,
                () -> -modifyAxis(controller.getLeftY()), // Axes are flipped here on purpose
                () -> -modifyAxis(controller.getLeftX()),
                () -> -modifyAxis(controller.getRightX())
        ));

        this.driverOI.getResetGyroButton().onTrue(new InstantCommand(() -> {
          this.drivetrain.zeroGyroscope();
        }, this.drivetrain));
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
