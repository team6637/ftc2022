package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivetrainSubsystem extends SubsystemBase {

    private final Motor leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor;
    private final MecanumDrive mecanum;
    RevIMU imu;
    Telemetry telemetry;
    static final double trackWidth = 12.25;
    static final double ticksPerInch = 29.3;
    static final double deadWheelOffset = 2.4;
    HolonomicOdometry holOdom;
    OdometrySubsystem odometry;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFrontMotor = new Motor(hardwareMap, "leftFrontMotor");
        rightFrontMotor = new Motor(hardwareMap, "rightFrontMotor");
        leftRearMotor = new Motor(hardwareMap, "leftRearMotor");
        rightRearMotor = new Motor(hardwareMap, "rightRearMotor");

        leftFrontMotor.setInverted(false);
        leftFrontMotor.encoder.setDirection(Motor.Direction.REVERSE);
        rightFrontMotor.setInverted(true);
        leftRearMotor.setInverted(false);
        rightRearMotor.setInverted(false);

        mecanum = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();
        this.telemetry = telemetry;

        leftFrontMotor.setDistancePerPulse(ticksPerInch);
        rightFrontMotor.setDistancePerPulse(ticksPerInch);
        leftRearMotor.setDistancePerPulse(ticksPerInch);

         holOdom = new HolonomicOdometry(
                 () -> leftFrontMotor.encoder.getDistance(),
                rightFrontMotor::getDistance,
                leftRearMotor::getDistance,
                trackWidth, deadWheelOffset
        );

        odometry = new OdometrySubsystem(holOdom);

    }

    public void stop() {
        mecanum.driveRobotCentric(0.0, 0.0, 0.0);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mecanum.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        mecanum.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, getHeading(), false);
    }

    public double getHeading() {
        return imu.getRotation2d().getDegrees();
    }

    public double getAbsoluteHeading() {
        return imu.getAbsoluteHeading();
    }

    public int getLeftPosition() {
        return leftFrontMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightFrontMotor.getCurrentPosition();
    }

    public int getDeadWheelPosition() {
        return leftRearMotor.getCurrentPosition();
    }

    public Pose2d getPose() {
       return odometry.getPose();
    }

    public OdometrySubsystem getOdometry() {
        return odometry;
    }

    public MecanumDrive getMecanum() {
        return mecanum;
    }

    @Override
    public void periodic() {
        super.periodic();
        telemetry.addData("heading", getHeading());
        telemetry.addData("absolute heading", getAbsoluteHeading());
        telemetry.addData("drive left position", getLeftPosition());
        telemetry.addData("drive right position", getRightPosition());
        telemetry.addData("dead wheel position", getDeadWheelPosition());
        telemetry.addData("x position", getPose().getX());
        telemetry.addData("y position" , getPose().getY());
        telemetry.update();
    }
    public void resetEncoder() {
        leftFrontMotor.encoder.reset();
        rightFrontMotor.encoder.reset();
        leftRearMotor.encoder.reset();
        rightRearMotor.encoder.reset();
    }
}
