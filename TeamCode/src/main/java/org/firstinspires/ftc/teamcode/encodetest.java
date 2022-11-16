package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class encodetest extends LinearOpMode {
    public void runOpMode() {
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor lb = hardwareMap.dcMotor.get("lb");
        DcMotor rb = hardwareMap.dcMotor.get("rb");
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");




        waitForStart();
        while (opModeIsActive()){

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            // We want to start the bot at x: 10, y: -8, heading: 90 degrees
            Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                    .build();

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);


            telemetry.addData("Front Distance", " %.1f",  frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", lf.getCurrentPosition(), rf.getCurrentPosition(),
                    lb.getCurrentPosition(), rb.getCurrentPosition());
            telemetry.update();
        }

    }
}
