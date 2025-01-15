package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

@Autonomous(name = "auto2", group= "something")
public class auto2 extends LinearOpMode {
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;

    @Override
    public void runOpMode()  {
        frontLeft = hardwareMap.get(DcMotor.class, "motor0");
        backLeft = hardwareMap.get(DcMotor.class, "motor2");
        frontRight = hardwareMap.get(DcMotor.class, "motor1");
        backRight = hardwareMap.get(DcMotor.class, "motor3");

        // set correct motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //this is for any side on blue we get. 2 seconds is enough to travel to parking from any start position.

        moveForward(0.5, 500); // for some reason this goes right, works for us idc
        strafeLeft(0.5, 1500);
    }

    public void moveForward(double power, long time) {   //goes right
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRight(double power, long time) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnleft(double power, long time) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeRight(double power, long time) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeLeft(double power, long time) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
