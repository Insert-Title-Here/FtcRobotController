package teamcode.common;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Vector;

import teamcode.common.PurePursuit.MathFunctions;

public class RobotPositionStateUpdater {
    RobotPositionState state;
    public static class RobotPositionState {
        private NormalizedRGBA houseRGBA;
        private NormalizedRGBA conveyorRGBA;
        private Vector2D position;
        private double rotation;
        private long positionUpdateTime;
        private Vector2D velocity;
        private double angularVelocity;
        private double linearSlidePosition;
        public RobotPositionState(Vector2D position, Vector2D velocity, double rotation, double angularVelocity, double linearSlidePosition, NormalizedRGBA houseRGBA, NormalizedRGBA conveyorRGBA) {
            this.position = position;
            this.velocity = velocity;
            this.rotation = rotation;
            this.angularVelocity = angularVelocity;
            this.linearSlidePosition = linearSlidePosition;
        }
        public Vector2D getPosition() {
            return position;
        }
        public double getRotation() {
            return rotation;
        }
        public double getAngularVelocity() {
            return angularVelocity;
        }
        public Vector2D getVelocity() {
            return velocity;
        }
        public double getLinearSlidePosition(){
            return linearSlidePosition;
        }
        public NormalizedRGBA getHouseRGBA(){
            return houseRGBA;
        }
        public NormalizedRGBA getConveyorRGBA(){
            return conveyorRGBA;
        }
        public RobotPositionState copy() {
            return new RobotPositionState(this.position, this.velocity, this.rotation, this.angularVelocity, this.linearSlidePosition, this.houseRGBA, this.conveyorRGBA);
        }

        public String toString(){
            return "Position: "  + position.toString() + "\n" +
                    "Velocity: " + velocity.toString() + "\n" +
                    "rotation: " + rotation + "\n" +
                    "angular Velocity: " + angularVelocity + "\n";
        }

        public String loggingToString(){
            return position.getX() + "," + position.getY() +"," + velocity.getX() + velocity.getY() + "," + rotation + "," + angularVelocity;
        }

    }

    public RobotPositionStateUpdater() {
        this(new Vector2D(0,0), new Vector2D(0,0), 0, 0, 0, null, null);
    }
    public RobotPositionStateUpdater(Vector2D position, Vector2D velocity, double rotation, double angularVelocity, double linearSlidePosition, NormalizedRGBA houseRGBA, NormalizedRGBA conveyorRGBA) {
        state = new RobotPositionState(position, velocity, rotation, angularVelocity, linearSlidePosition, houseRGBA, conveyorRGBA);
    }
    public synchronized RobotPositionState getCurrentState() {
        return state.copy();
    }
    public void resetUpdateTime() {
        state.positionUpdateTime = System.nanoTime();
    }
    private double updatePositionTime() {
        long newTime = System.nanoTime();
        double elapsedNanos = newTime - state.positionUpdateTime;
        state.positionUpdateTime = newTime;
        return elapsedNanos/1000000.0;
    }
    public synchronized void updateDelta(double deltaX, double deltaY, double deltaPhi,
                                         double deltaVx, double deltaVy, double omega) {
        updatePositionTime();
        state.position = state.position.add(new Vector2D(deltaX, deltaY));
        state.rotation = MathFunctions.angleWrap(state.rotation + deltaPhi);
        state.velocity = new Vector2D(deltaVx, deltaVy);
        state.angularVelocity = omega;
    }

    public synchronized void updateState(double deltaX, double deltaY, double deltaPhi,
                                         double deltaVx, double deltaVy, double omega,
                                         double linearSlidePosition, NormalizedRGBA houseRGBA, NormalizedRGBA conveyorRGBA) {
        updatePositionTime();
        state.position = new Vector2D(deltaX, deltaY);
        state.rotation =  MathFunctions.angleWrap(deltaPhi);
        state.velocity = new Vector2D(deltaVx, deltaVy);
        state.angularVelocity = omega;
        state.linearSlidePosition = linearSlidePosition;
        state.houseRGBA = houseRGBA;
        state.conveyorRGBA = conveyorRGBA;
    }


}
