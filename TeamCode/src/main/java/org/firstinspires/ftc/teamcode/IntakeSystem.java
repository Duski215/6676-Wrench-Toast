package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest")
public class IntakeSystem extends OpMode {

    //intake here
    private Servo intakeClawServo;
    private Servo horizontalSlideLeft;
    private Servo horizontalSlideRight;
    private Servo passoverServoLeft;
    private Servo passoverServoRight;

    /*toggle works poorly because you have to be really quick with the input so it doesn't
     double on it. AKA it goes from being false to true and back to being false
     */

    boolean toggleExtend = false;
    boolean togglePassover = false;

    @Override
    public void init() {
        intakeClawServo = hardwareMap.get(Servo.class, "servo0");
        horizontalSlideLeft = hardwareMap.get(Servo.class, "servo2");
        horizontalSlideRight = hardwareMap.get(Servo.class, "servo3");
        passoverServoLeft = hardwareMap.get(Servo.class, "exPassoverServo3");//wrong config name?
        passoverServoRight = hardwareMap.get(Servo.class,"passoverServo5");

//        intakeClawServo.setDirection(Servo.Direction.REVERSE);
        horizontalSlideRight.setDirection(Servo.Direction.REVERSE); // this aint a typo it supposed to be right idk why
        passoverServoLeft.setDirection(Servo.Direction.REVERSE);

//        horizontalSlideRight.scaleRange(0.2,0.8);
    }

    @Override
    public void loop() {
        //open
        if (gamepad1.x) {
            intakeClawServo.setPosition(.21);
        }

        //close
        if (gamepad1.y) {
            intakeClawServo.setPosition(.4);
        }

        // extend
        if (gamepad1.a) {
            //SUBJECT TO CHANGE DUE TO MECHANICAL AND SERVO RANGE
            //also set servo range to the specific rotation we need

            horizontalSlideRight.setPosition(0.5);
            horizontalSlideLeft.setPosition(0.5);
       /*
       Should make this a sequence from returning to reset position to passing to outtake so we
       we don't need to press an input for each individual part
       */
        }

        // retract
        if (gamepad1.b) {
            horizontalSlideLeft.setPosition(0.2);
            horizontalSlideRight.setPosition(0.2);
        }

//        intake position
        //this part will turn the claw between its intake position and passover position

        if (gamepad1.dpad_up) {
            passoverServoLeft.setPosition(1);
            passoverServoRight.setPosition(1);
        }
        if (gamepad1.dpad_down){
            passoverServoRight.setPosition(.3);
            passoverServoLeft.setPosition(.3);
        }

        telemetry.addData("Horizontal Slide Left Position",horizontalSlideLeft.getPosition());
        telemetry.addData("Horizontal Slide Right Position",horizontalSlideRight.getPosition());
//        telemetry.addData("Passover Slide Left Position",passoverServoLeft.getPosition());
//        telemetry.addData("Passover Slide Right Position",passoverServoRight.getPosition());
        telemetry.update();
    }
}

