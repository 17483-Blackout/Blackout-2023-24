package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BlackoutTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;

        // Hardware Map
        DcMotor frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        DcMotor backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        DcMotor frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        DcMotor backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor intake2 = hardwareMap.dcMotor.get("intake2");
        Servo Servo1 = hardwareMap.servo.get("Servo1");
        Servo Servo2 = hardwareMap.servo.get("Servo2");
        Servo Servo3 = hardwareMap.servo.get("Servo3");
         blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        blinkinLedDriver.setPattern(pattern);
        DcMotor Slide = hardwareMap.dcMotor.get("Slide");


        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        boolean intakeToggle = false;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad2.left_stick_y; // Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftDrive.setPower(frontLeftPower);
            backLeftDrive.setPower(backLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backRightDrive.setPower(backRightPower);

            if(gamepad1.a){
                Servo3.setPosition(0);
                sleep(1000);
                Servo1.setPosition(.5);
                Servo2.setPosition(.5);
                sleep(500);
                Servo3.setPosition(1);
            }
            if(gamepad1.b){
                Servo3.setPosition(.5);
            }
            if(gamepad1.dpad_up){
                Slide.setTargetPosition(500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(.7);
                telemetry.addData("Encoder Position", 500);
            }
            if(gamepad1.dpad_right){
                Slide.setTargetPosition(1000);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(.7);
                telemetry.addData("Encoder Position", 100);
            }
            if(gamepad1.dpad_down){
                Slide.setTargetPosition(1500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(.7);
                telemetry.addData("Encoder Position", 1500);
            }
            if (gamepad1.right_bumper) {
                intakeToggle = !intakeToggle;
            }
            if (intakeToggle) {
                intake.setPower(1);
                intake2.setPower(1);
            }
            else {
                intake.setPower(0);
                intake2.setPower(0);
            }

            telemetry.update();
        }
    }
}
