package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;


@Autonomous(name="First Roadrunner Auton")
public class FirstRoadrunnerAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx slideMotor1;
        DcMotorEx slideMotor2;
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servo = hardwareMap.servo.get("clawupdown");
        Servo servo1 = hardwareMap.servo.get("clawopenclose");
        CRServo crServo = hardwareMap.crservo.get("Slide3");
        slideMotor1 = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        slideMotor2 = hardwareMap.get(DcMotorEx.class, "Slide Motor 2");

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        crServo.setPower(-0.2);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        //raises servo claw
                        .stopAndAdd(new ServoAction(servo, 0.75))
                        //closes servo claw
                        .stopAndAdd(new ServoAction(servo1, 1))
                        .build());
        //trajectories
        Pose2d beginPose = new Pose2d(0, 0, 0);
        drive.updatePoseEstimate();

        TrajectoryActionBuilder tofirstpoint = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(9, -20));

        TrajectoryActionBuilder tosecondpoint = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(16, -20));

        waitForStart();

        int slideTargetPosition = 3130;
        slideMotor1.setTargetPosition(slideTargetPosition);
        slideMotor2.setTargetPosition(slideTargetPosition);

        slideMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor1.setPower(0.9);
        slideMotor2.setPower(0.9);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(8)
                        .build()
        );

        sleep(1000);

        crServo.setPower(1);

        sleep(750);

        crServo.setPower(0);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new ServoAction(servo, 0.75))
                        .stopAndAdd(new ServoAction(servo1, 0))
                        .build());

        sleep(500);

        crServo.setPower(-1);

        sleep(1000);

        slideMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slideMotor1.setTargetPosition(0);
        slideMotor2.setTargetPosition(0);

        crServo.setPower(0);

        sleep(1000);


//first point action finished ^

        Actions.runBlocking(
                tofirstpoint.build()
        );

        sleep(500);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(9, -20, 0))
                        .turn(-Math.PI/2)
                        .build()
        );

        sleep(500);

        crServo.setPower(1);

        sleep(800);

        crServo.setPower(0);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new ServoAction(servo, 0))
                        .waitSeconds(0.5)
                        .stopAndAdd(new ServoAction(servo1, 1))
                        .waitSeconds(0.5)
                        .stopAndAdd(new ServoAction(servo, 1))
                        .build());

        crServo.setPower(-1);

        sleep(1000);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(12, -20, 0))
                        .turn(Math.PI/4)
                        .build()
        );

        slideMotor1.setTargetPosition(slideTargetPosition);
        slideMotor2.setTargetPosition(slideTargetPosition);

        slideMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        sleep(2000);

        crServo.setPower(1);

        sleep(1000);

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new ServoAction(servo, 0.75))
                        .stopAndAdd(new ServoAction(servo1, 0))
                        .build());

        sleep(750);

        crServo.setPower(-1);

        sleep(1000);

        slideMotor1.setTargetPosition(0);
        slideMotor2.setTargetPosition(0);

        sleep(5000);

//second point actions finished ^

//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(16, -20, 0))
//                        .turn(Math.PI/4)
//                        .build()
//        );
//
//        Actions.runBlocking(
//                tosecondpoint.build()
//        );
//
//        crServo.setPower(1);
//
//        sleep(1000);
//
//        crServo.setPower(0);
//
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, 0, 0))
//                        .stopAndAdd(new ServoAction(servo, 0))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(new ServoAction(servo1, 1))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(new ServoAction(servo, 1))
//                        .build());
//
//        crServo.setPower(-1);
//
//        sleep(1000);
//
//        slideMotor1.setTargetPosition(slideTargetPosition);
//        slideMotor2.setTargetPosition(slideTargetPosition);
//
//        slideMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slideMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        sleep(2000);
//
//        crServo.setPower(1);
//
//        sleep(1000);
//
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, 0, 0))
//                        .stopAndAdd(new ServoAction(servo, 0.75))
//                        .stopAndAdd(new ServoAction(servo1, 0))
//                        .build());
//
//        sleep(750);
//
//        crServo.setPower(-1);
//
//        sleep(1000);
//
//        slideMotor1.setTargetPosition(0);
//        slideMotor2.setTargetPosition(0);








    }



    public static class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double p) {
            this.servo = s;
            this.position = p;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }
}




