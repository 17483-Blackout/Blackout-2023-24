package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class RedAutoBackdropSide extends OpMode {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    DcMotor frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
    DcMotor backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
    DcMotor frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
    DcMotor backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
    DcMotor intake = hardwareMap.dcMotor.get("intake");
    Servo Servo1 = hardwareMap.servo.get("Servo1");
    Servo Servo2 = hardwareMap.servo.get("Servo2");
    Servo Servo3 = hardwareMap.servo.get("Servo3");
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        Scalar lower = new Scalar(0, 150, 150); // the lower hsv threshold
        Scalar upper = new Scalar(10, 255, 255); // the upper hsv threshold
        double minArea = 80; // the minimum area for the detection to consider something as a prop

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) //Our Little buddy's eye
                .addProcessor(colourMassDetectionProcessor)
                .build();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
// look for instructions on how to have a switchable camera (switch back and forth between two cameras)
// or how to manually edit the exposure and gain, to account for different lighting conditions
// these may be extra features to work on to ensure that the robot performs consistently, even in different environments
    }

    /**
     * User-defined init_loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the init button is pressed and when the play button is pressed (or the
     * OpMode is stopped).
     * <p>
     * This method is optional. By default, this method takes no action.
     */
    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }

    /**
     * User-defined start method
     * <p>
     * This method will be called once, when the play button is pressed.
     * <p>
     * This method is optional. By default, this method takes no action.
     * <p>
     * Example usage: Starting another thread.
     */
    @Override
    public void start() {
// shuts down the camera once the match starts. Big Brother sees no more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

// gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

// now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
// if it is unfound it will just guess that it will be in the middle
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }


// now we can use recordedPropPosition to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                telemetry.addLine("Left");
                TrajectorySequence Left = drive.trajectorySequenceBuilder (new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(13, -35, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(8, -35))
                        .addTemporalMarker(1.3, () -> {
                            Servo3.setPosition(.9);
                        })
                        .waitSeconds(1)
                        .addTemporalMarker(2.3, () -> {
                            Servo1.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(13,-35))
                        .lineToLinearHeading(new Pose2d(13,-60, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -29, Math.toRadians(0)))
                        .addTemporalMarker(7.5, () -> {
                            Servo3.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(9.5, () -> {
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-65, -35))
                        .addTemporalMarker(15.5, () -> {
                            Servo3.setPosition(.2);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(17.5, () -> {
                            intake.setPower(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(19.5, () -> {
                            Servo1.setPosition(0);
                            Servo2.setPosition(0);
                            intake.setPower(0);
                        })
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(0)))
                        .addTemporalMarker(25.5, () -> {
                            Servo3.setPosition(.8);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(27.5, () -> {
                            Servo1.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(50, -60))
                        .build();
                drive.followTrajectorySequence(Left);
// code to do if we saw the prop on the left
                break;
            case UNFOUND:
                telemetry.addLine("No Object Detected");
                TrajectorySequence Unfound = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(13, -30))
                        .addTemporalMarker(1.3, () -> {
                            Servo3.setPosition(.9);
                        })
                        .waitSeconds(1)
                        .addTemporalMarker(2.3, () -> {
                            Servo1.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(13, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -29, Math.toRadians(0)))
                        .addTemporalMarker(7.5, () -> {
                            Servo3.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(9.5, () -> {
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-65, -35))
                        .addTemporalMarker(15.5, () -> {
                            Servo3.setPosition(.2);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(17.5, () -> {
                            intake.setPower(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(19.5, () -> {
                            Servo1.setPosition(0);
                            Servo2.setPosition(0);
                            intake.setPower(0);
                        })
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(0)))
                        .addTemporalMarker(25.5, () -> {
                            Servo3.setPosition(.8);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(27.5, () -> {
                            Servo1.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(50, -60))
                        .build();
                drive.followTrajectorySequence(Unfound);
                // we also just added the unfound case here to do fallthrough intstead of the overriding method above
            case MIDDLE:
                telemetry.addLine("Middle");
                TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(13, -30))
                        .addTemporalMarker(1.3, () -> {
                            Servo3.setPosition(.9);
                        })
                        .waitSeconds(1)
                        .addTemporalMarker(2.3, () -> {
                            Servo1.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(13, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -29, Math.toRadians(0)))
                        .addTemporalMarker(7.5, () -> {
                            Servo3.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(9.5, () -> {
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-65, -35))
                        .addTemporalMarker(15.5, () -> {
                            Servo3.setPosition(.2);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(17.5, () -> {
                            intake.setPower(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(19.5, () -> {
                            Servo1.setPosition(0);
                            Servo2.setPosition(0);
                            intake.setPower(0);
                        })
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(0)))
                        .addTemporalMarker(25.5, () -> {
                            Servo3.setPosition(.8);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(27.5, () -> {
                            Servo1.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(50, -60))
                        .build();
                drive.followTrajectorySequence(Middle);
// code to do if we saw the prop on the middle
                break;
            case RIGHT:
                telemetry.addLine("Right");
                TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToConstantHeading(new Vector2d(20, -40))
                        .addTemporalMarker(1.3, () -> {
                            Servo3.setPosition(.9);
                        })
                        .waitSeconds(1)
                        .addTemporalMarker(2.3, () -> {
                            Servo1.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(20, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -29, Math.toRadians(0)))
                        .addTemporalMarker(7.5, () -> {
                            Servo3.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(9.5, () -> {
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-65, -35))
                        .addTemporalMarker(15.5, () -> {
                            Servo3.setPosition(.2);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(17.5, () -> {
                            intake.setPower(.5);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(19.5, () -> {
                            Servo1.setPosition(0);
                            Servo2.setPosition(0);
                            intake.setPower(0);
                        })
                        .lineToConstantHeading(new Vector2d(-55, -35))
                        .lineToConstantHeading(new Vector2d(-55, -60))
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToLinearHeading(new Pose2d(40, -35, Math.toRadians(0)))
                        .addTemporalMarker(25.5, () -> {
                            Servo3.setPosition(.8);
                        })
                        .waitSeconds(2)
                        .addTemporalMarker(27.5, () -> {
                            Servo1.setPosition(.5);
                            Servo2.setPosition(.5);
                        })
                        .lineToConstantHeading(new Vector2d(40, -60))
                        .lineToConstantHeading(new Vector2d(50, -60))
                        .build();
                drive.followTrajectorySequence(Right);
// code to do if we saw the prop on the right
                break;
        }
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

    }

    /**
     * User-defined stop method
     * <p>
     * This method will be called once, when this OpMode is stopped.
     * <p>
     * Your ability to control hardware from this method will be limited.
     * <p>
     * This method is optional. By default, this method takes no action.
     */
    @Override
    public void stop() {
// this closes down the portal when we stop the code
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
}
//Vision actually makes me want to die
//Like literally this took me hours to figure out just how to make the pipeline
//And then I had to figure out how to get road runner into it
//But now I can sleep :)