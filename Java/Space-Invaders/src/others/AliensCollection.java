import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * This program is an AliensCollection object. Collection of Aliens.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-27
 */
public class AliensCollection implements Shooter {

    private List<List<Alien>> aliensCollection = new ArrayList<>();
    /**
     * When the player looses a life, the aliens reset their position back to.
     * the top
     */
    private double originalSpeed = 0;
    private List<List<Alien>> originalCollection = new ArrayList<>();
    private double commonSpeed = 0; // of all the aliens in the collection.
    private boolean onShield = false;

    /**
     * @param newSpeed - new common speed to each Alien in this collection.
     */
    public AliensCollection(double newSpeed) {
        this.commonSpeed = newSpeed;
        this.aliensCollection.clear();
        this.aliensCollection = new ArrayList<>();
        this.initializeCollection(); // calls to helper method.
        this.setSpeedOfCollection(newSpeed);
        this.setDefault(newSpeed, this.aliensCollection);

    }

    /**
     * @param aliensCollect - List<List<Alien>> collection of Aliens.
     * @param newSpeed      - new common speed to each Alien in this collection.
     */
    public AliensCollection(List<List<Alien>> aliensCollect, double newSpeed) {
        this.aliensCollection = aliensCollect;
        this.commonSpeed = newSpeed;
        this.initializeCollection(); // calls to helper method.
        this.setDefault(newSpeed, aliensCollect);
    }

    /**
     * @param common     - the common starting velocity.
     * @param collection - the collection in the start.
     */
    private void setDefault(double common, List<List<Alien>> collection) {

        this.originalSpeed = common;
        // this.originalCollection.addAll(this.aliensCollection);
        for (int i = 0; i < collection.size(); i++) {
            this.originalCollection.add(new ArrayList<Alien>(5));
            for (int j = 0; j < collection.get(0).size(); j++) {
                // Rectangle r = new Rectangle(new Point(100 + 50 * i + 30, 80 +
                // 60 * j), 40, 30);
                // Alien current = new Alien(r, 1, Color.BLACK);
                // currentEnemy.setLocation(i, j);
                Alien current = collection.get(i).get(j);
                this.originalCollection.get(i).add(current);
            }
        }
        // this.setCollectionToStart(); // ** need to remove.
    }

    /**
     * @return the start speed before changes.
     */
    public double getStartSpeed() {
        if (this.originalSpeed == 0) {
            System.out.println("original speed is null");
            return 0;
        }
        return this.originalSpeed;
    }

    /**
     * @return the collection before changes.
     */
    public List<List<Alien>> getStartCollection() {
        return this.originalCollection;
    }

    /**
     * This method initialized the collection.
     */
    private void initializeCollection() {

        this.aliensCollection.clear();

        for (int i = 0; i < 10; i++) {
            this.aliensCollection.add(new ArrayList<Alien>(5));
            for (int j = 0; j < 5; j++) {
                Rectangle r = new Rectangle(new Point(100 + 50 * i + 30, 80 + 60 * j), 40, 30);
                Alien current = new Alien(r, 1, Color.BLACK);
                // currentEnemy.setLocation(i, j);
                this.aliensCollection.get(i).add(current);
            }
        }
        // this.aliensCollection.addAll(collection);
    }

    /**
     * @param gameLevel - GameLevel object to enter the Alien to him.
     * @param bRemover  - BlockRemover in order we have the ability to remove the
     *                  Alien, in a case that the player shot on him.
     * @param sTracking - ScoreTrackingListener in order to add score when the player
     *                  shot in Alien.
     */
    public void addToGame(GameLevel gameLevel, BlockRemover bRemover, ScoreTrackingListener sTracking) {
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(0).size(); h++) {
                Alien current = this.aliensCollection.get(i).get(h);
                if (current != null) {
                    current.addAlienToGame(gameLevel);
                    current.addHitListener(sTracking);
                    current.addHitListener(bRemover);
                }
            }
        }
    }

    /**
     * @param newSpeed - new speed to set in our collection. Update speed in each
     *                 Alien in this Collection.
     */
    public void setSpeedOfCollection(double newSpeed) {
        this.commonSpeed = newSpeed;
        this.eachSpeedChanger();
    }

    /**
     * helper method that changes in each Alien his speed according the new one.
     */
    private void eachSpeedChanger() {
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(0).size(); h++) {
                Alien current = this.aliensCollection.get(i).get(h);
                if (current != null) {
                    current.setSpeed(this.commonSpeed);
                }
            }
        }
    }

    /**
     * This method move each Alien in our collection according his speed, that
     * must be the common speed.
     */
    public void moveCollection() {

        int count = 0;
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int j = 0; j < this.aliensCollection.get(i).size(); j++) {
                if (this.aliensCollection.get(i).get(j) != null) {
                    count++;
                }
            }
        }
        if (count == 0) {
            return;
        }
        Alien chosenEnemy = null;
        int i = 1;
        if (this.commonSpeed > 0) {

            chosenEnemy = this.getMostRight();
            // Now we found the closest Alien check if we can move.
            if (this.commonSpeed < 800 - chosenEnemy.getCollisionRectangle().getUpperLeft().getX()
                    - chosenEnemy.getCollisionRectangle().getWidth()) {
                for (int k = 0; k < this.aliensCollection.size(); k++) {
                    for (int j = 0; j < this.aliensCollection.get(k).size(); j++) {
                        if (this.aliensCollection.get(k).get(j) != null) {
                            this.aliensCollection.get(k).get(j).moveAlien();
                        }
                    }
                }
            } else {
                // Need to change direction and height

                this.setSpeedOfCollection(this.commonSpeed * -1 * 1.1);
                this.setCollectionHeight();

                // this.setHeightDown(this.commonSpeed);

            }
        } else {
            // Moves left.
            // We need to find the enemy in this column.

            chosenEnemy = this.getMostLeft();
            // Now we found the closest enemy check if we can move.
            if (Math.abs(this.commonSpeed) < chosenEnemy.getCollisionRectangle().getUpperLeft().getX()) {
                for (int k = 0; k < this.aliensCollection.size(); k++) {
                    for (int j = 0; j < this.aliensCollection.get(i).size(); j++) {
                        if (this.aliensCollection.get(k).get(j) != null) {
                            this.aliensCollection.get(k).get(j).moveAlien();
                        }
                    }
                }
            } else {
                this.setSpeedOfCollection(this.commonSpeed * -1 * 1.1);
                this.setCollectionHeight();
                // this.setHeightDown(this.commonSpeed);

            }
        }
    }

    /**
     * @return the most left Alien in this Collection. Find where is the first
     * index of a column that does not empty and return one from his
     * Aliens.
     */
    public Alien getMostLeft() {
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(i).size(); h++) {
                // in this way passing on columns.
                Alien current = this.aliensCollection.get(h).get(i);
                if (current != null) {
                    return current;
                }
            }
        }
        System.out.println("Null was returned from getMostLeft()");
        return null;
    }

    /**
     * @return the lowest Alien.
     */
    public Alien getMinAlien() {
        double minHeight = 0;
        int[] minIndexes = new int[2];
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int j = 0; j < this.aliensCollection.get(i).size(); j++) {
                if (this.aliensCollection.get(i).get(j) != null) {
                    if (this.aliensCollection.get(i).get(j).getCollisionRectangle().getUpperLeft().getY()
                            + this.aliensCollection.get(i).get(j).getCollisionRectangle().getHeight() > minHeight) {
                        minHeight = this.aliensCollection.get(i).get(j).getCollisionRectangle().getUpperLeft().getY()
                                + this.aliensCollection.get(i).get(j).getCollisionRectangle().getHeight();
                        minIndexes[0] = i;
                        minIndexes[1] = j;
                    }
                }
            }
        }
        return this.aliensCollection.get(minIndexes[0]).get(minIndexes[1]);
    }

    /**
     * @return the most right Alien in this Collection.
     */
    public Alien getMostRight() {
        for (int h = this.aliensCollection.get(0).size() - 1; h >= 0; h--) {
            for (int i = this.aliensCollection.size() - 1; i >= 0; i--) {
                // in this way passing on columns.
                Alien current = this.aliensCollection.get(i).get(h);
                if (current != null) {
                    return current;
                }
            }
        }
        System.out.println("Null was returned from getMostLeft()");
        return null;

    }

    /**
     * @param list - input of List<List<Alien>> to get the lowest from.
     * @return the lowest Alien in the collection. This uses us for: When the
     * lowest alien in the formation reaches the height of the shields
     * (or where the shields were, if all shields are destroyed), the
     * player looses a life.
     */
    public Alien getLowest(List<List<Alien>> list) {
        // need to start from the last row, therefore:
        for (int i = list.size() - 1; i >= 0; i--) {
            for (int h = 0; h < list.get(i).size(); h++) {
                // in this way passing on columns.
                Alien current = list.get(i).get(h);
                if (current != null) {
                    return current;
                }
            }
        }
        System.out.println("null was returned when trying getLowest() method");
        return null;
    }

    /**
     * @param coulmn - to check lowest into.
     * @param coulmnNumber - the number of the coulmn.
     * @return the lowest Alien that is not null in the input coulmn.
     */
    public Alien getLowestInCoulmn(List<Alien> coulmn, int coulmnNumber) {
        for (int i = coulmn.size() - 1; i >= 0; i--) {
            System.out.println(i);
            if (coulmn.get(i) != null && !coulmn.get(i).getCollisionRectangle().isHitted()) {
                return coulmn.get(i);
            }
        }
        System.out.println(" getLowestInCoulmn return null ");
        System.out.println(" trying shot from " + coulmnNumber + " coulmn");
        return null;
    }

    /**
     * Helper function.
     *
     * @param alienRow List<Alien> - list of Aliens in the same row (index 0).
     * @return The Most Left Alien In this row, it means in this list of Aliens.
     */
    private Alien getMostLeftInRow(List<Alien> alienRow) {
        double xMin = alienRow.get(0).getCollisionRectangle().getUpperLeft().getX();
        double currentResult;
        Alien minAlien = alienRow.get(0);
        for (Alien current : alienRow) {
            if (current != null) {
                currentResult = current.getCollisionRectangle().getUpperLeft().getX();
                if (currentResult < xMin) {
                    xMin = currentResult;
                    minAlien = current;
                }
            }
        }
        return minAlien;
    }

    @Override
    /**
     * Every 0.5 seconds a random column of aliens is chosen to shoot, the
     * lowest alien on that column will release a shot.
     */
    public void shot(GameLevel gameLevel) {
        Random randomGenerator = new Random();
        int numColumn = randomGenerator.nextInt(this.aliensCollection.size());
        while (this.isNull(this.aliensCollection.get(numColumn))) {
            numColumn = randomGenerator.nextInt(this.aliensCollection.size());
        }
        List<Alien> chosenColumn = this.aliensCollection.get(numColumn);
        double minHeight = 0;
        int indOfLowestEnemy = 0;
        // Search for the lowest Enemy in this column.
        for (int i = 0; i < chosenColumn.size(); i++) {
            if (chosenColumn.get(i) != null && !chosenColumn.get(i).isHitted()) {
                if (chosenColumn.get(i).getCollisionRectangle().getUpperLeft().getY() > minHeight) {
                    minHeight = chosenColumn.get(i).getCollisionRectangle().getUpperLeft().getY();
                    indOfLowestEnemy = i;
                }
            }
        }
        chosenColumn.get(indOfLowestEnemy).shot(gameLevel);
    }

    /**
     * set the collection to the data we has in the start.
     */
    public void setCollectionToStart() {

        this.setSpeedOfCollection(this.originalSpeed);
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int j = 0; j < this.aliensCollection.get(i).size(); j++) {
                if (this.aliensCollection.get(i).get(j) != null) {
                    this.aliensCollection.get(i).get(j).setRecOfAlienToOriginal();
                    //this.aliensCollection.get(i).get(j).setSpeed(this.originalSpeed);
                }
            }
        }
    }

    /**
     * Set to this.onShield = true.
     */
    public void setToOnShield() {
        this.onShield = true;
    }

    /**
     * @return true if on shield. OtherWise false.
     */
    public boolean ifOnShield() {
        if (this.onShield) {
            // set to false.
            this.onShield = false;
            return true;
        }
        return false;
    }

    /**
     * @param howMuchGoDown - double number = how much we need to go down with our
     *                      collection.
     */
    public void setHeightDown(double howMuchGoDown) {
        if (this.onShield) {
            return;
        }
        Alien minEnemy = this.getMinAlien();
        if (minEnemy.getCollisionRectangle().getUpperLeft().getY() + minEnemy.getCollisionRectangle().getHeight()
                + howMuchGoDown > 500 + 1) {
            this.setToOnShield();
            return;
        }

        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(i).size(); h++) {
                Alien current = this.aliensCollection.get(i).get(h);
                current.setAlienLocation(current.getCollisionRectangle().getUpperLeft().getX(),
                        current.getCollisionRectangle().getUpperLeft().getX() + howMuchGoDown);
            }
        }
    }

    /**
     * @param coulmnOfAliens List<Alien> for checking.
     * @return true if this coulmn of aliens is empty. Otherwise -> false.
     */
    private boolean checkIfCoulmnIsEmpty(List<Alien> coulmnOfAliens) {
        for (Alien current : coulmnOfAliens) {
            if (!current.getCollisionRectangle().isHitted()) {
                return false; // there is at least one block therefore not
                // empty.
            }
        }
        System.out.println("empty");
        return true;
    }

    /**
     * @param index - index we want his list.
     * @return List<Alien> of this index coulmn.
     */
    private List<Alien> getCoulmn(int index) {
        // System.out.println("your index is: " + index);
        List<Alien> coulmn = new ArrayList<>();
        for (List<Alien> current : this.aliensCollection) {
            coulmn.add(current.get(index));
        }
        return coulmn;
    }

    /**
     * @param lst list of enemies.
     * @return true if all of them are null, false otherwise.
     */
    public boolean isNull(List<Alien> lst) {
        // System.out.println("Size is: " + lst.size());
        for (int i = 0; i < lst.size(); i++) {
            if (lst.get(i) != null && !lst.get(i).isHitted()) {
                return false;
            }
        }
        return true;
    }

    /**
     * set height of collection.
     */
    private void setCollectionHeight() {


        double height = Math.abs(this.commonSpeed);

        if (this.onShield) {
            return;
        }
        Alien minEnemy = this.getMinAlien();
        if (minEnemy.getCollisionRectangle().getUpperLeft().getY() + minEnemy.getCollisionRectangle().getHeight()
                + height > 500 + 1) {
            this.setToOnShield();
            return;
        }

        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(i).size(); h++) {
                Alien current = this.aliensCollection.get(i).get(h);
                Rectangle currentRec = this.aliensCollection.get(i).get(h).getCollisionRectangle();
                current.setAlienLocation(currentRec.getUpperLeft().getX(), currentRec.getUpperLeft().getY() + height);
            }
        }
    }

    /**
     * @return number of Aliens in this collection.
     */
    private int getNumberOfStartAliens() {
        int counter = 0;
        for (int i = 0; i < this.aliensCollection.size(); i++) {
            for (int h = 0; h < this.aliensCollection.get(i).size(); h++) {
                if (this.aliensCollection.get(i).get(h) != null
                        && !this.aliensCollection.get(i).get(h).getCollisionRectangle().isHitted()) {
                    counter += 1;
                }
            }
        }
        return counter;
    }
}
