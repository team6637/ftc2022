package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class RobotContainer extends CommandOpMode {

    private GamepadEx controller;
    private GamepadEx controller2;
    private GripperSubsystem gripperSubsystem;
    private DrivetrainSubsystem m_drive;
    private LiftSubsystem liftSubsystem;
    private VisionSubsystem visionSubsystem;


    @Override
    public void initialize() {
        gripperSubsystem = new GripperSubsystem(hardwareMap);
        m_drive = new DrivetrainSubsystem(hardwareMap, telemetry);
        liftSubsystem = new LiftSubsystem(hardwareMap, telemetry);
        m_drive.resetEncoder();
        controller = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);
        m_drive.setDefaultCommand(new DriveCommand(controller, hardwareMap, m_drive, telemetry));
        visionSubsystem = new VisionSubsystem(hardwareMap,telemetry);


        // Load Controllers
        loadOperatorControls(controller2);
    }
    public void loadOperatorControls(GamepadEx controller) {

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    gripperSubsystem.open();
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    gripperSubsystem.close();
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(0);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(1250);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(2050);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(2900);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(0);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    liftSubsystem.setSetpoint(500);
                })
        );

    }

}
