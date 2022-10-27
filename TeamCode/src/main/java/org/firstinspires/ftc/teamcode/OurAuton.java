package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GripperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Autonomous
public class OurAuton extends CommandOpMode {
    private GripperSubsystem gripperSubsystem;
    private DrivetrainSubsystem m_drive;
    private LiftSubsystem liftSubsystem;
    //private VisionSubsystem visionSubsystem;

    @Override
    public void initialize() {
        gripperSubsystem = new GripperSubsystem(hardwareMap);
        m_drive = new DrivetrainSubsystem(hardwareMap, telemetry);
        liftSubsystem = new LiftSubsystem(hardwareMap, telemetry);
        m_drive.resetEncoder();
        //visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);

        Waypoint p1 = new StartWaypoint(0.0, 0.0);
        Waypoint p2 = new GeneralWaypoint(24.0,0.0,0.5,0.5,0.0);
        Waypoint p3 = new EndWaypoint(
                36, 0, 0, 0.5,
                0.5, 30, 0.8, 1
        );

        PurePursuitCommand ppCommand = new PurePursuitCommand(
                m_drive.getMecanum(), m_drive.getOdometry(),
                p1, p2,p3
        );


        schedule(ppCommand);

        //telemetry.addData("Color is ", visionSubsystem.getResult());
    }
}
