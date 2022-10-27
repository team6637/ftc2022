package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {

    GamepadEx controller;
    DrivetrainSubsystem m_drive;

    double angleSetpoint = 0.0;
    double angleKp = 0.04;
    double angleError = 0.0;
    double currentAngle;
    Telemetry t;
    //PIDController anglePID;

    public DriveCommand(GamepadEx c, HardwareMap hardwareMap, DrivetrainSubsystem drivetrainSubsystem, Telemetry t) {
        this.t = t;
        controller = c;
        m_drive = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);

        //anglePID = new PIDController(angleKp, 0.0, 0.0);
    }

    @Override
    public void execute() {
        double forward = controller.getLeftY() * -1.0 * 0.8;

        double strafe = controller.getLeftX() * -1.0;
        if(Math.abs(strafe) < 0.3) {
            strafe = 0.0;
        }

        strafe = strafe * 0.8;

        currentAngle = m_drive.getAbsoluteHeading();

        controller.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    angleSetpoint = 0.0;
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    angleSetpoint = 90.0;
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    angleSetpoint = 180.0;
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    angleSetpoint = -90.0;
                })
        );

        if(angleSetpoint == 180.0) {
            if(currentAngle < 0.0) {
                angleError = (angleSetpoint - Math.abs(currentAngle)) * -1.0;
            } else {
                angleError = angleSetpoint - currentAngle;
            }

        } else {
            angleError = angleSetpoint - currentAngle;
        }


        double turn = angleError * angleKp;

        t.addData("drive angle error", angleError);
        t.addData("drive angle setpoint", angleSetpoint);
        t.addData("drive angle kp", angleKp);


        m_drive.driveFieldCentric(strafe, forward, turn);
    }
}
