package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;


@Config
@Autonomous(name="Strafe Test", group="Autonomous")


public class Strafe extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -30));

        waitForStart();

        Actions.runBlocking(
                toSubmersible.build()
        );

       sleep(1000);


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, -30, 0))
                        .turn(-Math.PI/2)
                        .build()
        );

        sleep(2000);


    }

}