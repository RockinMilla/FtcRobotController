package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Red", group="Robot")
public class AutoDriveRed extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");

        telemetry.addData("This code was last updated", "10/16/2025, 4:18 pm"); // Todo: Update this date when the code is updated

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.getPinpointPosition();
        r.driveToPos(-500, -500, 0); // backwards, strafe left
        r.driveToPos(-500, -500, 45); // turn towards goal
        sleep(500); // pew pew pew
        r.driveToPos(-500, -600, 45); // strafe right
        r.driveToPos(-1000, -600, 90); // turn right
        r.driveToPos(-1000, -200, 90); // forward (intake balls)
        sleep(500); // slurp up balls
        r.driveToPos(-500, -500, 90); // backward
        r.driveToPos(-500, -500, 45); // turn towards goal
        sleep(500); // pew pew pew
        r.driveToPos(-1600, -500, 45); // strafe right
        r.driveToPos(-1600, -500, 90); // turn right
        r.driveToPos(-1600, -200, 90); // forward (intake balls)
        sleep(500); // slurp up balls
        r.driveToPos(-500, -500, 90); // backward, strafe left
        r.driveToPos(-500, -500, 45); // turn towards goal
        sleep(500); // pew pew pew
        // turn towards gate
        // park ready to activate gate
        RobotLog.vv("Rockin' Robots", "Test Done");
        sleep(4 * 1000);
    }
}