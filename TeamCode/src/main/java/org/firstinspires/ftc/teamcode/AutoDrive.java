package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        // Send a telemetry message to signify that the robot is ready to run;
        telemetry.addData("Autonomous Ready", "You can press start");
        telemetry.addData("This code was last updated", "04/1/2025, 3:52 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.configureOtos(); // Todo: we don't have an OTOS. change this to deadwheels

        RobotLog.vv("Rockin' Robots", "Quick Test Code");
        for(int i = 0; i < 3; i++) {
            r.driveToLoc(5, 0, 0);
            sleep(1000);
            r.driveToLoc(0, 5, 0);
            sleep(1000);
            r.driveToLoc(-5, 0, 0);
            sleep(1000);
            r.driveToLoc(0, -5, 0);
            sleep(1000);
            r.driveToLoc(0,0,0);
        }

        RobotLog.vv("Rockin' Robots", "Test Done");
        telemetry.addData("Autonomous lasted", runtime.toString());
    }
}