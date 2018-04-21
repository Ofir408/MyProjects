
//package threeass;

//import secondass.Ball;
//import secondass.Velocity;
/**
 * That's Collidable interface.
 * The Collidable interface will be used by things that can be collided with
 */
public interface Collidable {

    /**
     * @return the "collision shape" of the object.
     */
    Rectangle getCollisionRectangle();

    /**
     * Notify the object that we collided with it at collisionPoint with
     * a given velocity.
     * The return is the new velocity expected after the hit (based on
     * the force the object inflicted on us).
     * @param collisionPoint - the point where it's collided with the rectangle.
     * @param currentVelocity - the current velocity before the hit.
     * @param ball - the ball that hits.
     * @return the new velocity expected after the hit.
     */
    Velocity hit(Ball ball, Point collisionPoint, Velocity currentVelocity);
}
