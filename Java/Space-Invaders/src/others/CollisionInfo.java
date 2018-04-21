//package threeass;

//import secondass.Point;

/**
 * That's CollisionInfo class.
 * that has the point at which the collision occurs.
 * and the collidable object involved in the collision.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-28
 */
public class CollisionInfo {
    /**
     * the collisionPoint.
     */
    private Point collisionPoint;
    /**
     * the collisionObject.
     */
    private Collidable collisionObject;

    /**
     * This is constructor function of CollisionInfo object.
     * The Input:
     *
     * @param pointOfcollision  - the point at which the collision occurs,
     *                          it's a  Point object.
     * @param objectOfcollision - the collidable object involved
     *                          in the collision, Collidable object.
     */
    public CollisionInfo(final Point pointOfcollision,
                         final Collidable objectOfcollision) {
        this.collisionObject = objectOfcollision;
        this.collisionPoint = pointOfcollision;
    }

    /**
     * @return the point at which the collision occurs (Point object).
     */
    public Point collisionPoint() {
        return this.collisionPoint;
    }

    /**
     * @return the collidable object involved in the collision.
     */
    public Collidable collisionObject() {
        return this.collisionObject;
    }

}
