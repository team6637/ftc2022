package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {

    public Motor liftMotor;
    private final PIDController pidController;
    private final double kp = 0.01;
    Telemetry telemetry;
    private int setpoint = 0;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        liftMotor = new Motor(hardwareMap,"lift");
        liftMotor.setInverted(true);
        pidController = new PIDController(kp,0.0,0.0);
        this.telemetry = telemetry;
    }

    public int getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void up() {
        if(getCurrentPosition() > 2800) {
            stop();
        } else {
            liftMotor.set(0.5);
        }
    }

    public void down() {
        if(getCurrentPosition() <= 0) {
            stop();
        } else {
            liftMotor.set(-0.5);
        }
    }

    public void stop() {
        liftMotor.set(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        telemetry.addData("lift setpoint", setpoint);
        telemetry.addData("lift current position", getCurrentPosition());
        //telemetry.update();
        double power = pidController.calculate(getCurrentPosition(), setpoint);
        liftMotor.set(power);

    }

    public void setSetpoint(int setpoint) {
        this.setpoint = setpoint;
    }
}
