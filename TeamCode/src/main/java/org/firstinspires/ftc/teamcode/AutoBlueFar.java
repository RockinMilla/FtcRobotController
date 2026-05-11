package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Far", group="Robot")
public class AutoBlueFar extends LinearOpMode {
    //final ElapsedTime runtime = new ElapsedTime();
    private double launcherVelocity = 1140; // Change this when changing launcher velocity

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "5/10/2026, 5:50 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.intakePower(0.5);
        r.launcherVelocity(launcherVelocity);

        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos(-100, 800, 95, 10, 2, 4);
        r.driveToPos(-1250, 800, 95, 10,2, 4, true);
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(-1200, 300, 170, 10, 2, 4);
        r.driveToPos(-1250, 20, 170, 10, 2, 4, true);
        shootBalls(r);

        // Park
        r.driveToPos(-500, 200, 70); 
        r.launcherVelocity(0);
        r.intakePower(0);
        r.lightsOff();
    }

    private void shootBalls(RockinBot r)
    {
        r.driveToPos(50, 150, 34, 10, 1.5, 5);
        r.waitForLaunchers(launcherVelocity);
        sleep(500);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}
