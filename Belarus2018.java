package org.firstinspires.ftc.teamcode;


import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Timer;
import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.hypot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo.Direction;



@TeleOp (name = "Belarus2018")
public class Belarus2018 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor left = null; // контрол хаб порт 2, движение
    private DcMotor right = null; // контрол хаб порт 3, движение
    private DcMotor lift = null; // контрол хаб порт 1, лифт
    private DcMotor leftgetup = null; // экспаншн порт 2, подъемник
    private DcMotor rightgetup = null; // экспаншн порт 1, подъемник
    private DcMotor conveyor = null; // экспаншн порт 3, конвейер
    private DcMotor WindTurbine = null; // контрол порт 0, ветряк


    private CRServo forwardwind = null; // контрол порт 1, выдвижение
    private CRServo beatingcube = null; // контрол порт 2, поворот куба
    private Servo container = null; // экспаншн порт 4, закрывалка контейнера
    private CRServo claw = null; // экспаншн порт 5,  клешня
    private CRServo unload = null; // контрол порт 1, отвечает за выгрузку
    private CRServo clawup = null; // экспаншн порт 5,  клешня


    static final double COUNTS_PER_MOTOR_REV = 100; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 1; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    int turbinebackward = 0;
    int turbineforward = 0;
    int claw1 = 0;
    int containeropen = 0;
    int liftrunmode = 0;
    int claw2 = 0;
    int level = 1;
    int loading = 0;


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        leftgetup = hardwareMap.get(DcMotor.class, "leftgetup");
        rightgetup = hardwareMap.get(DcMotor.class, "rightgetup");
        WindTurbine = hardwareMap.get(DcMotor.class, "WindTurbine");
        lift = hardwareMap.get(DcMotor.class, "lift");

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");


        container = hardwareMap.get(Servo.class, "container");
        forwardwind = hardwareMap.get(CRServo.class, "forwardwind");
        claw = hardwareMap.get(CRServo.class, "claw");
        unload = hardwareMap.get(CRServo.class, "unload");
        beatingcube = hardwareMap.get(CRServo.class, "beatingcube");
        clawup = hardwareMap.get(CRServo.class, "clawup");

        leftgetup.setDirection(DcMotor.Direction.REVERSE);


        leftgetup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightgetup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftgetup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightgetup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftgetup.getCurrentPosition(),
                rightgetup.getCurrentPosition());
        telemetry.update();


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.update();

            if (gamepad2.y) {
                switch (level) {
                    case 1:
                        level = 3;
                        encoderDrivegetup(1,13,13,5);
                        break;
                    case 2:
                        level = 3;
                        encoderDrivegetup(1,3,3,2);
                        break;
                    case 3:
                        break;
                }
                while (gamepad2.y);
            }

            if (gamepad2.b){
                switch (level){
                    case 1:
                        level = 2;
                        encoderDrivegetup(1,10,10,4);
                        break;
                    case 2:
                        break;
                    case 3:
                        level = 2;
                        encoderDrivegetup(1,-1.5,-1.5,2);
                        break;
                }
                while (gamepad2.b);
            }

            if (gamepad2.a){
                switch (level){
                    case 1:
                        break;
                    case 2:
                        leftgetup.setPower(0);
                        rightgetup.setPower(0);
                        level = 1;
                        encoderDrivegetup(1, -5, -5, 2);
                        leftgetup.setPower(0);
                        rightgetup.setPower(0);
                        break;
                    case 3:
                        leftgetup.setPower(0);
                        rightgetup.setPower(0);
                        level = 1;
                        encoderDrivegetup(1, -10, -10, 2);
                        leftgetup.setPower(0);
                        rightgetup.setPower(0);
                        break;
                }
                while (gamepad2.a);
            }

            if (level != 1){
                leftgetup.setPower(0.05);
                rightgetup.setPower(0.05);
            }

            if (gamepad2.dpad_up) {
                switch (turbineforward) {
                    case 0:
                        turbineforward = 1;
                        forwardwind.setPower(1);
                        break;
                    case 1:
                        turbineforward = 0;
                        forwardwind.setPower(0);
                        break;
                }
                while (gamepad2.dpad_up) ;
            }

            if (gamepad2.dpad_down) {
                switch (turbinebackward) {
                    case 0:
                        turbinebackward = 1;
                        forwardwind.setPower(-1);
                        break;
                    case 1:
                        turbinebackward = 0;
                        forwardwind.setPower(0);
                        break;
                }
                while (gamepad2.dpad_down) ;
            }

            if (gamepad2.dpad_left) {
                WindTurbine.setPower(-1);
            } else if (gamepad2.dpad_right) {
                WindTurbine.setPower(1);
            } else
                WindTurbine.setPower(0);


            if (gamepad2.x) {
                switch (liftrunmode) {
                    case 0:
                        liftrunmode = 1;
                        leftgetup.setPower(0);
                        rightgetup.setPower(0);
                        telemetry.addData("liftmode", "is Auto");
                        telemetry.update();
                        break;
                    case 1:
                        liftrunmode = 0;
                        leftgetup.setPower(gamepad2.left_stick_y);
                        rightgetup.setPower(gamepad2.right_stick_y);
                        telemetry.addData("liftmode", "is Manual");
                        telemetry.update();
                        break;

                }
                while (gamepad2.x) ;
            }

            if (gamepad2.right_bumper) {
                switch (containeropen) {
                    case 0:
                        containeropen = 1;
                        container.setPosition(1);
                        break;
                    case 1:
                        containeropen = 0;
                        container.setPosition(0.2);
                        break;
                }
                while (gamepad1.right_bumper) ;
            }

            if (gamepad1.b){
                switch(loading){
                    case 0:
                        loading = 1;
                        lift.setPower(1);
                        unload.setPower(1);
                        beatingcube.setPower(1);
                        break;
                    case 1:
                        loading = 0;
                        lift.setPower(0);
                        unload.setPower(0);
                        beatingcube.setPower(0);
                        break;
                }
                while (gamepad1.b);
            }

            if (gamepad1.left_bumper) {
                switch (claw2) {
                    case 0:
                        claw2 = 1;
                        claw.setPower(1);
                        break;
                    case 1:
                        claw2 = 0;
                        claw.setPower(0);
                        break;
                    case 2:
                        claw2 = 1;
                        claw.setPower(1);
                        break;
                }
                while (gamepad1.left_bumper);
            }

            if (gamepad1.right_bumper){
               switch (claw2){
                   case 0:
                       claw2 = 2;
                       claw.setPower(-1);
                       break;
                   case 1:
                       claw2 = 2;
                       claw.setPower(-1);
                       break;
                   case 2:
                       claw2 = 0;
                       claw.setPower(0);
                       break;

               }
               while(gamepad1.right_bumper);
            }

            if (gamepad1.dpad_up){
                switch(claw1){
                    case 0:
                        claw1 = 1;
                        clawup.setPower(1);
                        break;
                    case 1:
                        claw1 = 0;
                        clawup.setPower(0);
                        break;
                    case 2:
                        claw1 = 1;
                        clawup.setPower(1);
                        break;

                }
                while(gamepad1.dpad_up);
            }

            if (gamepad1.dpad_down){
                switch(claw1){
                    case 0:
                        claw1 = 2;
                        clawup.setPower(-1);
                        break;
                    case 1:
                        claw1 = 2;
                        clawup.setPower(-1);
                        break;
                    case 2:
                        claw1 = 0;
                        clawup.setPower(0);
                        break;

                }
                while(gamepad1.dpad_down);
            }




        }
    }

    public void encoderDrivegetup ( double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS){
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget = leftgetup.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightgetup.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftgetup.setTargetPosition(newLeftTarget);
            rightgetup.setTargetPosition(newRightTarget);


            leftgetup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightgetup.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            leftgetup.setPower(Math.abs(speed));
            rightgetup.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftgetup.isBusy() && rightgetup.isBusy())) {

            }


            leftgetup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightgetup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            leftgetup.setPower(0.05);
            rightgetup.setPower(0.05);


        }
    }
}
