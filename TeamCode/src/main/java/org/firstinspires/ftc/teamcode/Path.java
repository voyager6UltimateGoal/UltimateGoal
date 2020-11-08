package org.firstinspires.ftc.teamcode;

public class Path {
    public M move;
    public double speed;
    public double arg;
    public Path(M _move, D direction, double _speed, double _arg) {
        this.move = _move;
        if(direction == D.FORWARD) {
            this.speed = Math.abs(_speed);
            if(_move != M.ROTATE) {
                this.arg = Math.abs(_arg);
            } else {
                this.arg = _arg;
            }
        } else {
            this.speed = -Math.abs(_speed);
            if(_move != M.ROTATE) {
                this.arg = -Math.abs(_arg);
            } else {
                this.arg = _arg;
            }
        }
    }
}