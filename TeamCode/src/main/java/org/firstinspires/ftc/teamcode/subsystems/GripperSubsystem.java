package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GripperSubsystem extends SubsystemBase {

    private double minAngle;
    private double maxAngle;

    ServoEx rightServo;
    ServoEx leftServo;

    public GripperSubsystem (HardwareMap hardwareMap) {

        rightServo = new SimpleServo(hardwareMap, "gripperRight", minAngle, maxAngle);
        leftServo = new SimpleServo(hardwareMap, "gripperLeft", minAngle, maxAngle);

        leftServo.setInverted(true);

    }

    public void open() {
        rightServo.setPosition(1.0);
        leftServo.setPosition(1.0);
    }

    public void close() {
        rightServo.setPosition(0.0);
        leftServo.setPosition(0.0);
    }



}
