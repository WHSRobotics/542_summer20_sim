package util.swervetotarget;

import util.Position;

public class SwerveWaypoint {

    Position position;
    double tangentialVelocity;

    public SwerveWaypoint(Position position, double tangentialVelocity){
        this.position = position;
        this.tangentialVelocity = tangentialVelocity;
    }


    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public double getTangentialVelocity() {
        return tangentialVelocity;
    }

    public void setTangentialVelocity(double tangentialVelocity) {
        this.tangentialVelocity = tangentialVelocity;
    }


}
