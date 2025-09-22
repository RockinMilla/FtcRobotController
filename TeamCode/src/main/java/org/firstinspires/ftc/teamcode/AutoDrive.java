package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Driver");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "09/21/2025, 5:34 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.getPinpointPosition();
        r.driveToPos(500, -500, 0); // backwards, strafe left, turn towards goal
        r.driveToPos(500, -500, -45); // backwards, strafe left, turn towards goal
        sleep(3000); // pew pew pew
        r.driveToPos(500, -600, -45); // turn right, strafe right
        r.driveToPos(1000, -600, -90); // turn right, strafe right
        r.driveToPos(1000, -200, -90); // forward (intake balls)
        sleep(2000); // slurp up balls
        r.driveToPos(500, -500, -90); // backward, turn towards goal
        r.driveToPos(500, -500, -45); // backward, turn towards goal
        sleep(3000); // pew pew pew
        r.driveToPos(500, -1300, -45); // turn right, strafe right
        r.driveToPos(500, -1300, -90); // turn right, strafe right
        r.driveToPos(200, -1300, -90); // forward (intake balls)
        sleep(2000); // slurp up balls
        r.driveToPos(500, -500, -90); // backward, strafe left, turn towards goal
        r.driveToPos(500, -500, -45); // backward, strafe left, turn towards goal
        sleep(3000); // pew pew pew
        // turn towards gate
        // park ready to activate gate
        RobotLog.vv("Rockin' Robots", "Test Done");
        telemetry.addData("Autonomous lasted", runtime.toString());
        sleep(4 * 1000);
    }
}