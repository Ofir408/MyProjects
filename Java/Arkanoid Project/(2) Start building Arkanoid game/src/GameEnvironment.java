//package threeass;

import java.util.ArrayList;
import java.util.List;

//import secondass.Line;
//import secondass.Point;

/**
 * That's GameEnvironment class.
 * The ball will know the game environment,
 * and will use it to check for collisions and direct its movement.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-04-28
 */
public class GameEnvironment {

    /**
     * list of collidables in the environment.
     */
    private List<Collidable> collidableList = new ArrayList<Collidable>();

    /**
     * add the given collidable to the environment.
     *
     * @param c - Collidable object that we will add him to the collection.
     */
    public void addCollidable(final Collidable c) {
        this.collidableList.add(c);
    }

    /**
     * @return collidableList - list of collidables.
     */
    public List<Collidable> getCollidableList() {
        return this.collidableList;
    }

    /**
     * Assume an object moving from line.start() to line.end().
     * If this object will not collide with any of the collidables
     * in this collection, return null. Else, return the information
     * about the closest collision that is going to occur.
     *
     * @param trajectory the object movement.
     * @return an object - CollisionInfo that has the information
     * about the closest collision that is going to occur.
     */
    public CollisionInfo getClosestCollision(final Line trajectory) {
        // List of CollisionInfo object to save the
        // the point at which the collision occurs
        // and the collidable object involved in the collision
        List<CollisionInfo> collisionInfoList = new ArrayList
                <CollisionInfo>();
        for (int i = 0; i < this.collidableList.size(); i += 1) {
            List<Point> intersPointList = this.collidableList.
                    get(i).getCollisionRectangle().
                    intersectionPoints(trajectory);
            if (!intersPointList.isEmpty()) {
                // it means that there is intersection so there
                // is a collision. Store all the collision
                // in a list of intersection points.
                // convert them to list of the CollisionInfo.
                for (int a = 0; a < intersPointList.size();
                     a += 1) {
                    collisionInfoList.add(
                            new CollisionInfo(
                                    intersPointList.get(a),
                                    this.collidableList.
                                            get(i)));
                } // finished added points for one
                //  collidable object.
            }
        }
        // here we finished passing all the collidable things we have.
        if (collisionInfoList.isEmpty()) {
            return null;
        } else {
            // it means collide with any of the collidables
            //in this collection so call helper function checking
            // about the closest collision that is going to occur

            return this.closestCollision(
                    trajectory, collisionInfoList);
        }
    }

    /**
     * Getting collisionInfoList (list of collision info objects)
     * and then return the information
     * about the closest collision that is going to occur.
     *
     * @param trajectory        - the object movement.
     * @param collisionInfoList - list of CollisionInfo objects.
     * @return CollisionInfo object that has closest collision that is
     * going to occur.
     */
    public CollisionInfo closestCollision(final Line trajectory,
                                          final List<CollisionInfo> collisionInfoList) {
        // initilized the first to be the closet for beggining.
        CollisionInfo infoClosest = collisionInfoList.get(0);
        for (int i = 1; i < collisionInfoList.size(); i += 1) {
            CollisionInfo tempCollisionInfo =
                    collisionInfoList.get(i);
            Point tempCloser = trajectory.closestPointToLine(
                    infoClosest.collisionPoint(),
                    tempCollisionInfo.collisionPoint());

            if (tempCloser.equals(tempCollisionInfo.
                    collisionPoint())) {
                // we need to replace because there is more
                // closer point to the start of line.
                infoClosest = tempCollisionInfo;
            }
            // now we have the temp close point between the line
            // and the collision in place i in the arrayList
            // of collisionInfo.
        }
        return infoClosest;
    }


}
