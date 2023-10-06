package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Red_Backstage_45")
public class A_R_Back extends LinearOpMode{
    DcMotor FLDrive = null; // standard motor declarations
    DcMotor FRDrive = null;
    DcMotor BLDrive = null;
    DcMotor BRDrive = null;
    IMU imu = null;

    public void runOpMode() throws InterruptedException {


        double desiredHeading = 0;


        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
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

        //TODO: WRITE YOUR AUTO CODE HERE!
        driveStraight(10);
        turnToAngle(90.0);
        driveStraight(20);
        turnToAngle(180);



    }

    void driveStraight(double inches){
        double i = inches;
        int ticks = inchesToTicks(i);

        int encoderError = 10;

        while ( isBusy() ) {
            if ( Math.abs(FLDrive.getCurrentPosition())- FLDrive.getTargetPosition() < encoderError) {
                FLDrive.setPower(0);
            } if ( Math.abs(FRDrive.getCurrentPosition())- FRDrive.getTargetPosition() < encoderError) {
                FRDrive.setPower(0);
            } if ( Math.abs(BLDrive.getCurrentPosition())- BLDrive.getTargetPosition() < encoderError) {
                BRDrive.setPower(0);
            } if ( Math.abs(BRDrive.getCurrentPosition())- BRDrive.getTargetPosition() < encoderError) {
                BRDrive.setPower(0);
            }
        }
        stopMove();

    }

    public boolean isBusy() {
        if ( FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy() ) {
            return true;
        } else {
            return false;
        }
    }


    int inchesToTicks(double inches){
        return (int) inches*134;
    }


    void turnToAngle(double angle) {
        double botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double rotate = botHeadingDeg - angle; // algorithm for automatic turning
        rotate += 540;
        rotate = (rotate % 360) - 180;

        double rx = rotate / 70;
        FLDrive.setPower(rx);
        BLDrive.setPower(rx);
        BLDrive.setPower(-rx);
        BRDrive.setPower(-rx);

        while (Math.abs(botHeadingDeg - angle) > 10) {// 10 degree margin
            botHeadingDeg = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    public void stopMove() {
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
    }

}
