//package threeass;

import java.awt.Color;

import biuoop.DrawSurface;
import biuoop.GUI;
import biuoop.Sleeper;
//import secondass.Ball;
//import secondass.Point;

/**
 * This program is a Game class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-03
 */

public class Game {

    /**
     * sprites is the name of class: SpriteCollection.
     * that has the collection of the sprites
     */
    private SpriteCollection sprites;

    /**
     * environment is the name of class: GameEnvironment
     * that has the collection of the Collidables.
     */
    private GameEnvironment environment;

    /**
     * paddle object, for the player in order to move and
     * hit balls in our game.
     */
    private Paddle paddle;

    /**
     * This is constructor function of Game object.
     *
     * @param s - SpriteCollection, collection of sprite interfaces.
     * @param g - GameEnvironment the environment of the game.
     * @param p - the paddle for the user.
     */
    public Game(final SpriteCollection s, final GameEnvironment g,
                final Paddle p) {
        this.sprites = s;
        this.environment = g;
        this.paddle = p;
    }

    /**
     * This is another constructor function of Game object.
     *
     * @param p - the paddle for the user.
     */
    public Game(final Paddle p) {
        this.paddle = p;
    }

    /**
     * build game with no parameters.
     */
    public Game() {
    }


    /**
     * add the given Collidable to the collection of Collidable.
     *
     * @param c - Collidable object that we will add
     *          him to the collection.
     */

    public void addCollidable(final Collidable c) {
        this.environment.addCollidable(c);
    }

    /**
     * add the given Sprite to the collection of spirits.
     *
     * @param s - Sprite object that we will add him to
     *          the collection.
     */
    public void addSprite(final Sprite s) {
        this.sprites.addSprite(s);

    }

    /**
     * Initialize a new game: create the Blocks and Ball (and Paddle)
     * and add them to the game.
     *
     * @param gui - GUI, helps to draw.
     */
    public void initialize(final GUI gui) {

        this.sprites = new SpriteCollection();
        this.environment = new GameEnvironment();
        Ball ball1 = new Ball(new Point(250, 300), 7,
                Color.red);
        Ball ball2 = new Ball(new Point(50, 50), 7,
                Color.red);
        ball1.setVelocity(7, 4);
        ball2.setVelocity(7, 4);
        Color c = Color.GRAY;

        Block[] blocksArray = {new Block(new Rectangle(
                new Point(0, 20), 600, 20), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 20, 600),
                        1, c),
                new Block(new Rectangle(new Point(
                        580, 600), 20, 600), 1, c),
                new Block(new Rectangle(new Point(
                        0, 600), 600, 20), 1, c)};
        c = Color.gray;
        for (int i = 0; i < 440; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 140), 40, 20),
                    2, c).addToGame(this);
        }

        // another row - first row - orange color
        c = Color.ORANGE;
        for (int i = 0; i < 400; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 160), 40, 20),
                    1, c).addToGame(this);
        }
        // second row -  red color
        c = Color.RED;
        for (int i = 0; i < 360; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 180), 40, 20),
                    1, c).addToGame(this);
        }
        c = Color.darkGray;
        for (int i = 0; i < 320; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 200), 40, 20),
                    1, c).addToGame(this);
        }
        c = Color.CYAN;
        for (int i = 0; i < 280; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 220), 40, 20),
                    1, c).addToGame(this);
        }
        c = Color.GREEN;
        for (int i = 0; i < 240; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 240), 40, 20),
                    1, c).addToGame(this);
        }
        c = Color.PINK;
        for (int i = 0; i < 200; i += 40) {
            new Block(new Rectangle(new Point(
                    500 - i, 260), 40, 20),
                    1, c).addToGame(this);
        }

        Rectangle r = new Rectangle(new Point(280, 550),
                50, 20);

        biuoop.KeyboardSensor keyboard =
                gui.getKeyboardSensor();
        this.paddle = new Paddle(keyboard, r, 20, Color.YELLOW);
        this.paddle.addToGame(this);
        // add new rows
        // r1.addToGame(this);

        Sprite s1 = ball1;
        Sprite s2 = ball2;
        ball1.setGameEnvironment(this.environment);
        ball2.setGameEnvironment(this.environment);
        this.addSprite(s1);
        this.addSprite(s2);
        // s2.addToGame(this);
        // s1.addToGame(this);


        for (int i = 0; i < blocksArray.length; i += 1) {
            Block block = blocksArray[i];
            block.addToGame(this);
        }
    }

    /**
     * Run the game -- start the animation loop.
     *
     * @param gui - for the DrawSurface.
     */
    public void run(final GUI gui) {
        //...

        Sleeper sleeper = new Sleeper();
        int framesPerSecond = 60;
        int millisecondsPerFrame = 1000 / framesPerSecond;
        Rectangle rec1 = new Rectangle(new Point(0, 600), 600, 600);
        Block b = new Block(rec1, 1, Color.BLUE);

        while (true) {
            DrawSurface d = gui.getDrawSurface();
            b.drawBackground(d, b.getColor());
            long startTime = System.currentTimeMillis(); // timing


            this.sprites.drawAllOn(d);
            gui.show(d);
            this.sprites.notifyAllTimePassed();


            // timing
            long usedTime = System.currentTimeMillis() - startTime;
            long milliSecondLeftToSleep =
                    millisecondsPerFrame - usedTime;
            if (milliSecondLeftToSleep > 0) {
                sleeper.sleepFor(milliSecondLeftToSleep);
            }
        }
    }

    /**
     * @param args none.
     *             run the game.
     */
    public static void main(final String[] args) {
        GUI gui = new GUI("Arkanoid", 600, 600);
        Game game = new Game();
        game.initialize(gui);
        game.run(gui);
    }
} // OF CLASS

