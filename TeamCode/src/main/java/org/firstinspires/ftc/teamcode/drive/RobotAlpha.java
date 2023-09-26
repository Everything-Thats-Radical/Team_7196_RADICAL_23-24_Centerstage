package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Math.signum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp (name = "V1.0", group = "Iterative Opmode")
public class RobotAlpha extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FLDrive = null; // standard motor declarations
        DcMotor FRDrive = null;
        DcMotor BLDrive = null;
        DcMotor BRDrive = null;

        //Write numerical variables here
        double desiredHeading = 0;

        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();
        double yHeading = 0;
        double xHeading = -90;
        double bHeading = 90;
        double aHeading = 180;


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //TODO: Finish write tele-op

            double speedMultiplier;

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.y) { // automatic turning commands
                desiredHeading = yHeading;
            }
            if (gamepad1.x) {
                desiredHeading = xHeading;
            }
            if (gamepad1.b) {
                desiredHeading = bHeading;
            }
            if (gamepad1.a) {
                desiredHeading = aHeading;
            }

            boolean slowMode = gamepad1.right_bumper;
            if (slowMode) {
                speedMultiplier = .5;
            } else {
                speedMultiplier = 1.0;
            }

            double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rotate = botHeadingDeg - desiredHeading; // algorithm for automatic turning
            rotate += 540;
            rotate = (rotate % 360) - 180;
            rx += rotate/-70;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // bot heading for field centric
            // Rotate the movement direction counter to the bot's rotation
            // Changes x and y from robot centric drive to field-centric
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX - rx) / denominator; // standard mecanum wheel formulas
            double backLeftPower = (rotY - rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX + rx) / denominator;
            double backRightPower = (rotY + rotX + rx) / denominator;

            FLDrive.setPower(frontLeftPower * speedMultiplier); // set power to wheels
            BLDrive.setPower(backLeftPower * speedMultiplier);
            FRDrive.setPower(frontRightPower * speedMultiplier);
            BRDrive.setPower(backRightPower * speedMultiplier);


        }

    }
}