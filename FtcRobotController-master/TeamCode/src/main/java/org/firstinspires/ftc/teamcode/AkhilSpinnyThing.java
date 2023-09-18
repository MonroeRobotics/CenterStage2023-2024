public class AkhilSpinnyThing extends OpMode{
    DcMotor spinny;
    double motorPower = .5;

    @Override
    public void init() {
        //mapping motors onto motor objects
        spinny = hardwareMap.get(DcMotor.class, "spinny");

        //setting motors runmode
        spinny.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        spinny.setPower(motorPower);
    }

    @Override
    public void stop() {
        motorPower = 0;
        spinny.setPower(motorPower);
    }
}
