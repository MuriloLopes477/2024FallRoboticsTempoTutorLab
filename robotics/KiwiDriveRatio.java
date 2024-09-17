package edu.elon.robotics;

public class KiwiDriveRatio {
    public double powerLeft;
    public double powerAux;
    public double powerRight;

    private boolean isAuto;

    public KiwiDriveRatio(boolean isAuto) {
        this.isAuto = isAuto;
        powerRight = 0.0;
        powerLeft = 0.0;
        powerAux = 0.0;
    }
    public void computeRatio(double drive, double strafe, double turn) {
        if (isAuto) computeRatioAuto(drive, strafe, turn);
        else computeRatioTelop(drive,strafe,turn);

        double max = Math.max(Math.max(Math.max(Math.abs(powerLeft),Math.abs(powerRight)),1.0),powerAux);

        powerLeft  /= max;
        powerRight /= max;
        powerAux   /= max;
    }
    private void computeRatioAuto(double drive, double strafe, double turn) {
        powerLeft  =   0.58 * drive + strafe / 3.0 + turn / 3.0;
        powerRight = - 0.58 * drive + strafe / 3.0 + turn / 3.0;
        powerAux   =          - 2.0 * strafe / 3.0 + turn / 3.0;

    }

    private void computeRatioTelop(double drive, double strafe, double turn) {
        powerLeft  = -0.5 * strafe - Math.sqrt(3.0)/2.0 * drive + turn;
        powerRight = -0.5 * strafe + Math.sqrt(3.0)/2.0 * drive + turn;
        powerAux   = strafe + turn;


    }
}
