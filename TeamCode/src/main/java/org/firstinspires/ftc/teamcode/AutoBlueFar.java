package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Far", group="Robot")
public class AutoBlueFar extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "5/03/2026, 5:17 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();

        r.intakePower(0.5);
        r.launcherVelocity(1200); // Change this when changing launcher velocity

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(-100, 800, 90, 10, 2, 4);
        r.driveToPos(-1310, 800, 90, 10,2,4, true);
        r.intakePower(0.8);
        //sleep(100);
        //r.driveToPos( -1390, 150, 90, 10, 2, 1.5); // Pick up
        r.waitForLifter();
        sleep(100);

        shootBalls(r); // Shoot 2nd set of balls

        r.driveToPos(-500, 200, 70); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(50, 150, 28, 10, 1.5 ,5);
        r.waitForLaunchers(1200); // Change this when changing launcher velocity
        sleep(2000);
        r.intakePower(0.5);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}