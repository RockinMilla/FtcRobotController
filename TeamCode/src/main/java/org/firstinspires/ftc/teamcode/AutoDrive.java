package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    GoBildaPinpointDriver odo;
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        sleep(3 * 1000);

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "07/28/2025, 2:46 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {
            r.getPinpointPosition();
            RobotLog.vv("Rockin' Robots", "Callie move it");
            sleep(10 * 1000);
            r.getPinpointPosition();
            RobotLog.vv("Rockin' Robots", "Test Done");
            telemetry.addData("Autonomous lasted", runtime.toString());
            sleep(20 * 1000);
/*
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            RobotLog.vv("Rockin' Robots", "Position: " + data);
            r.driveToPos(0, 10000, 0); //todo
            //sleep(1000);
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            RobotLog.vv("Rockin' Robots", "Position: " + data);

            RobotLog.vv("Rockin' Robots", "Test Done");
            telemetry.addData("Autonomous lasted", runtime.toString()); */

        }
    }
}