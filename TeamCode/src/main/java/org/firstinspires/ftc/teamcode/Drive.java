public class MyFIRSTJavaOpMode extends LinearOpMode {
    DcMotor driveLeft;
    DcMotor driveRight;
    DcMotor shootwheel;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    Servo artifactstopper;
    ColorSensor color1;
    DistanceSensor distance1;
    BNO055IMU imu;
var turn,A,Dx,Dy,M,O,d,g,Red,Blue,myBno055imuParameters,robotOrientation,myVisionPortalBuilder,myVisionPortal,myAprilTagProcessorBuilder,myAprilTagProcessor,myAprilTagDetection,myAprilTagDetection2,myAprilTagDetections;
@Override
    public void runOpMode() {
        driveLeft = hardwareMap.get(DcMotor.class, "driveLeft");
        driveRight = hardwareMap.get(DcMotor.class, "driveRight");
        shootwheel = hardwareMap.get(DcMotor.class, "shootwheel");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");  
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        artifactstopper = hardwareMap.get(Servo.class, "artifactstopper");
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        myBno055imuParameters = (new BNO055IMU.Parameters());
        imu.initialize(myBno055imuParameters);
        BNO055IMU.Parameters.setAngleUnit(myBno055imuParameters, BNO055IMU.AngleUnit.DEGREES);
        BNO055IMU.Parameters.setAccelUnit(myBno055imuParameters, BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC);
        BNO055IMU.Parameters.setSensorMode(myBno055imuParameters, BNO055IMU.SensorMode.IMU);
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortal = (myVisionPortalBuilder.build());
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        O=0;
        d=0.75;
        while (opModeIsActive()) {
            turn=gamepad1.right_trigger-gamepad1.left_trigger;
            robotOrientation = (imu.getAngularOrientation());
            if (gamepad1.b) {
                O=robotOrientation;
            }
            robotOrientation-=O;
            A=180/Math.PI*Math.atan(gamepad1.left_stick_y/(gamepad1.left_stick_x+0.0000000001));
            if (gamepad1.left_stick_x<0) {
                A+=180;
            }
            if (A<0) {
                A+=360;
            }
            if (gamepad1.x) {
                d=1;
            }else {
                d=0.5;
            }
            A-=(360-robotOrientation);
            M=Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2));
            M=d * (3 * Math.pow(M,2) - 2 * Math.pow(M,3));
            Dx=M*Math.cos(A/180*Math.PI);
            Dy=M*Math.sin(A/180*Math.PI);
            frontLeftDrive.setPower(-Dx-Dy-turn);
            frontRightDrive.setPower(-Dx+Dy-turn);
            backLeftDrive.setPower(Dx-Dy-turn);
            backRightDrive.setPower(Dx+Dy-turn);
            myAprilTagDetections=(myAprilTagProcessor.getDetections());
            if (gamepad1.a) {
                artifactstopper.setPosition(0);
                shootwheel.setPower(0.8);
                sleep(250);
                artifactstopper.setPosition(0.2);
                sleep(200);
                shootwheel.setPower(0);
                sleep(1500);
            }
        }
    }
}