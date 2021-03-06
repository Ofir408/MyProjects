
//package threeass;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;
import biuoop.GUI;
import biuoop.KeyboardSensor;
import biuoop.Sleeper;
//import secondass.Ball;
//import secondass.Point;

/**
 * This program is a GameLevel class.
 *
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-05-03
 */

public class GameLevel implements Animation {

    /**
     * sprites is the name of class: SpriteCollection. that has the collection
     * of the sprites
     */
    private SpriteCollection sprites;

    /**
     * environment is the name of class: GameLevel that has the collection of
     * the Collidables.
     */
    private GameEnvironment environment;

    /**
     * paddle object, for the player in order to move and hit balls in our game.
     */
    private Paddle paddle;

    /**
     * Counter that has details about how much blocks remain, because blocks
     * remove when they have 0 hits - point.
     */
    private Counter counterBlocks;

    /**
     * Counter that has details about how much Balls remain in the game.
     */
    // private Counter counterBalls;

    /**
     * Counter that has details about the score of the player in this game.
     */
    private Counter counterScore;

    /**
     * Counter that has details about how much lives the player still have.
     */
    private Counter numberOfLives;
    /**
     * the GUI object.
     */
    private GUI gui;
    /**
     *
     */
    private AnimationRunner runner;
    /**
     * running is true if the game still run, Otherwise - false.
     */
    private boolean running;
    /**
     * Keyboard to get input from the user, like "P" in order to stop the game.
     */
    private KeyboardSensor keyboard;
    /**
     * information about this level.
     */
    private LevelInformation levelInformation;
    private AliensCollection aliensCollection;
    private List<Shield> shieldList = new ArrayList<>();
    private List<Ball> iterationBalls = new ArrayList<>();

    private HighScoresTable highScoresTable;

    /**
     * The time was passed from last shot of the Alien.
     */
    private double timePassedAlien = 0;
    private double numberOfLevel = 0; // level was passed.

    /**
     * @param l - LevelInformation.
     */
    public GameLevel(final LevelInformation l) {
        this.levelInformation = l;
    }

    /**
     * This is constructor function of GameLevelLevelLevelLevel object.
     *
     * @param s - SpriteCollection, collection of sprite interfaces.
     * @param g - GameLevel the environment of the game.
     * @param p - the paddle for the user.
     */
    public GameLevel(final SpriteCollection s, final GameEnvironment g, final Paddle p) {
        this.sprites = s;
        this.environment = g;
        this.paddle = p;
    }

    /**
     * This is another constructor function of GameLevelLevelLevelLevel object.
     *
     * @param s          - SpriteCollection, collection of sprite interfaces.
     * @param g          - GameLevel the environment of the game.
     * @param p          - the paddle for the user.
     * @param newCounter - Counter object to check how much blocks removed.
     */
    public GameLevel(final SpriteCollection s, final GameEnvironment g, final Paddle p, final Counter newCounter) {
        this.sprites = s;
        this.environment = g;
        this.paddle = p;
        this.counterBlocks = newCounter;
    }

    /**
     * This is another constructor function of GameLevelLevelLevelLevel object.
     *
     * @param p - the paddle for the user.
     */
    public GameLevel(final Paddle p) {
        this.paddle = p;
    }

    /**
     * This is another constructor function of GameLevelLevelLevelLevel object.
     *
     * @param c - the Counter.
     */
    public GameLevel(final Counter c) {
        this.counterBlocks = c;
    }

    /**
     * build game with no parameters.
     */
    public GameLevel() {
    }

    /**
     * @param level           - current LevelInformation.
     * @param keyboardSensor  - KeyboardSensor
     * @param animationRunner - AnimationRunner
     * @param livesCount      - Counter of lives.
     * @param scoreCount      - Counter of the score of the player.
     */
    public GameLevel(LevelInformation level, KeyboardSensor keyboardSensor, AnimationRunner animationRunner,
                     Counter livesCount, Counter scoreCount) {
        this.levelInformation = level;
        this.numberOfLives = livesCount;
        this.sprites = new SpriteCollection();
        this.environment = new GameEnvironment();
        this.runner = animationRunner;
        this.keyboard = keyboardSensor;
        this.counterScore = scoreCount;
        this.gui = this.runner.getGui();
    }

    /**
     * set high score table.
     *
     * @param newScoreTable - new score table to set.
     */
    public void setHighScoreTable(HighScoresTable newScoreTable) {
        this.highScoresTable = newScoreTable;
    }

    /**
     * add the given Collidable to the collection of Collidable.
     *
     * @param c - Collidable object that we will add him to the collection.
     */

    public void addCollidable(final Collidable c) {
        this.environment.addCollidable(c);
    }

    /**
     * add the given Sprite to the collection of spirits.
     *
     * @param s - Sprite object that we will add him to the collection.
     */
    public void addSprite(final Sprite s) {
        this.sprites.addSprite(s);
    }

    /**
     * @param newLevelInfo - new LevelInformation to set.
     */
    public void setLevelInformation(LevelInformation newLevelInfo) {
        this.levelInformation = newLevelInfo;
    }

    /**
     * @param s - Sprite to remove from this current GameLevel object.
     */
    public void removeSprite(final Sprite s) {
        this.sprites.removeSprite(s);
    }

    /**
     * @param c - Collidable to remove from this current GameLevel object.
     */
    public void removeCollidable(final Collidable c) {
        this.environment.removeCollidable(c);
    }

    /**
     * function that creates two balls and put them in the game.
     */
    public void createBalls() {

        if (this.levelInformation.levelName().equals("Zero level")) {
            List<Ball> bList = new ZeroLevelInformation().getFinalListOfBalls();
            enterBallToGame(bList);
        }

        if (this.levelInformation.levelName().equals("Direct hit")) {
            List<Ball> bList = new GameLevel1().getFinalListOfBalls();
            enterBallToGame(bList);
        } else if (levelInformation.levelName().equals("Wide Easy")) {
            // it means that it's the second level, therefore:
            List<Ball> ballsList = new GameLevel2().getFinalListOfBalls();
            enterBallToGame(ballsList);
        } else if (levelInformation.levelName().equals("Green 3")) {
            List<Ball> ballsList = new GameLevel3().getFinalListOfBalls();
            enterBallToGame(ballsList);
        } else if (levelInformation.levelName().equals("Final Four")) {
            List<Ball> ballsList = new GameLevel4().getFinalListOfBalls();
            enterBallToGame(ballsList);
        } else {
            enterBallToGame(this.getFinalListOfBalls());
        }
    }

    /**
     * function that removed the current Paddle and create new one.
     */
    public void createPaddle() {

        this.removeCollidable(this.paddle);
        this.removeSprite(this.paddle);
        // need to change when it's 800 x 600.
        int xOfPaddle = (800 - this.levelInformation.paddleWidth()) / 2;
        Rectangle r = new Rectangle(new Point(xOfPaddle, 550), this.levelInformation.paddleWidth(), 20);
        biuoop.KeyboardSensor keyboardd = this.gui.getKeyboardSensor();
        this.keyboard = keyboardd;
        if (this.levelInformation.levelName().equals("Wide Easy")) {
            this.paddle = new Paddle(keyboard, r, this.levelInformation.paddleSpeed(), Color.ORANGE);
            this.paddle.setGame(this);
            this.paddle.addToGame(this);
            return;
        }
        this.paddle = new Paddle(keyboard, r, this.levelInformation.paddleSpeed(), Color.YELLOW);
        this.paddle.setGame(this);
        this.paddle.addToGame(this);
    }

    /**
     * @return gameEnvironmantel.
     */
    public GameEnvironment getGameEnvironment() {
        return this.environment;
    }

    /**
     * @param newB - adding this ball to our list.
     */
    public void addBall(Ball newB) {
        this.iterationBalls.add(newB);
    }

    /**
     * remove balls from this iteration of the game.
     */
    public void removeIterationBalls() {
        for (Ball b : this.iterationBalls) {
            b.removeFromGame(this);
        }
        this.iterationBalls.clear();
        this.iterationBalls = new ArrayList<>();
    }

    /**
     * Initialize a new game: create the Blocks and Ball (and Paddle) and add
     * them to the game.
     */
    public void initialize() {

        BlockRemover bRemover = new BlockRemover(this, new Counter(49));
        this.counterBlocks = bRemover.getCounterRemainBlocks();

        // BallRemover ballsRemover = new BallRemover(this, new Counter(0));
        // this.counterBalls = ballsRemover.getCounterRemainBalls();
        ScoreTrackingListener sTracking = new ScoreTrackingListener(this.counterScore);
        // this.counterScore = sTracking.getScoreCounter();
        ScoreIndicator scoreIndicat = new ScoreIndicator(this.counterScore);
        // Run a game with 4 lives
        LivesIndicator livesInd = new LivesIndicator(this.numberOfLives);
        this.numberOfLives = livesInd.getLivesCounter();
        this.sprites = new SpriteCollection();
        this.environment = new GameEnvironment();

        // this.levelInformation = new GameLevel1(); ////////**************/////
        this.addSprite(this.levelInformation.getBackground());

        for (int i = 0; i < this.levelInformation.blocks().size(); i += 1) {
            Block currentBlock = this.levelInformation.blocks().get(i);
            currentBlock.addToGame(this);

            // in order not to it for the slides blocks.
            if (i > 4) {
                currentBlock.addHitListener(bRemover);
                this.counterBlocks.increaseOne();
                currentBlock.addHitListener(sTracking);
            }
        }

        // trying to add the Alien to the game
        this.aliensCollection = new AliensCollection(0.75 + 0.25 * this.numberOfLevel);
        this.aliensCollection.addToGame(this, bRemover, sTracking);
        this.aliensCollection.setSpeedOfCollection(0.75 + 0.25 * this.numberOfLevel);
        // this.setToDefaultCollection(); // **** remove it ****

        // add shield
        this.shieldList.add(new Shield(new Point(150, 500)));
        this.shieldList.add(new Shield(new Point(330, 500)));
        this.shieldList.add(new Shield(new Point(510, 500)));

        for (int i = 0; i < this.shieldList.size(); i++) {
            this.shieldList.get(i).removeShieldToGame(this, bRemover, sTracking);
            this.shieldList.get(i).removeShieldToGame(this, bRemover, sTracking);
            this.shieldList.get(i).removeShieldToGame(this, bRemover, sTracking);
            this.shieldList.get(i).addShieldToGame(this, bRemover, sTracking);
        }

        scoreIndicat.addToGame(this);
        livesInd.addToGame(this);
        NameLevelIndicator nameOfLevel = new NameLevelIndicator(this.levelInformation.levelName());
        nameOfLevel.addToGame(this);

    }

    /**
     * is in charge of stopping condition.
     *
     * @return true if the level should stop (because of no more balls, or no
     * more blocks). Otherwise, return false.
     */
    @Override
    public boolean shouldStop() {
        return !this.running;
    }

    /**
     * doOneFrame(DrawSurface) is in charge of the logic.
     *
     * @param d  - DrawSurface.
     * @param dt - the amount of seconds passed since the last call.
     */
    @Override
    public void doOneFrame(DrawSurface d, double dt) {

        int framesPerSecond = 60;
        int millisecondsPerFrame = 1000 / framesPerSecond;

        this.sprites.drawAllOn(d);
        this.sprites.notifyAllTimePassed(dt);

        Sleeper sleeper = new Sleeper();

        while (this.running) {

            d = this.gui.getDrawSurface();
            long startTime = System.currentTimeMillis(); // timing

            this.timePassedAlien += dt;

            this.aliensCollection.moveCollection();

            this.sprites.drawAllOn(d);

            this.gui.show(d);
            this.sprites.notifyAllTimePassed(dt);

            // timing
            long usedTime = System.currentTimeMillis() - startTime;
            long milliSecondLeftToSleep = millisecondsPerFrame - usedTime;
            if (milliSecondLeftToSleep > 0) {
                sleeper.sleepFor(milliSecondLeftToSleep);
            }

            if (this.counterBlocks.getValue() == 0) {
                this.numberOfLevel++;
                this.setLevelName();
                this.run();
            }

            if (this.keyboard.isPressed("p")) {
                this.runner.run(new PauseScreen(this.keyboard));
            }

            if (this.keyboard.isPressed(KeyboardSensor.SPACE_KEY) && this.paddle.canShotOneMore()) {
                this.paddle.shot(this);
            }

            // need to change the following after adding collection of Aliens.
            if (this.timePassedAlien >= 0.5) {
                // the Alien needs to shot new ball
                // this.alien.shot(this);
                // this.alien.setSpeed(10);
                // this.alien.moveAlien();
                this.aliensCollection.shot(this);
                this.timePassedAlien = 0;
            }

            if (this.paddle.ifPaddleHitted()) {
                // if(true){
                // if (this.numberOfLives.getValue() == 1) {
                // this.removeIterationBalls();
                this.running = false; // stop running.
                // }
            }
            // if (true) {
            if (this.aliensCollection.ifOnShield()) {
                this.numberOfLives.decreaseOne();
                // this.removeIterationBalls();
                // this.aliensCollection.set
                this.running = false; // stop running.
            }

        }
    }

    /**
     * Run the game -- start the animation loop.
     */
    public void playOneTurn() {
        boolean b = false;
        this.createPaddle();
        // added this line:
        // this.initialize();

        // this.createBalls();
        this.runner = new AnimationRunner(this.gui);
        this.runner.run(new CountdownAnimation(2, 3, this.sprites));

        this.running = true;

        // use our runner to run the current animation -- which is one turn of
        // the game.
        this.runner.run(this);


        if (this.numberOfLives.getValue() > 0) {
            this.removeIterationBalls();
            this.setToDefaultCollection();
            this.playOneTurn();
        }
    }

    /**
     * run.
     */
    public void run() {

        this.initialize();

        while (this.numberOfLives.getValue() > 0) {
            this.playOneTurn();
            this.numberOfLives.decreaseOne();
            // reset the game to defaults.
        }
    }

    /**
     * @param ballsList - list to enter to the level.
     */
    public void enterBallToGame(List<Ball> ballsList) {
    }

    /**
     * @return NumberOfLivesCounter
     */
    public Counter getNumberOfLivesCounter() {
        return this.numberOfLives;
    }

    /***
     * @return BlocksCounter.
     */
    public Counter getBlocksCounter() {
        return this.counterBlocks;
    }

    /**
     * @return list of balls initialized.
     */
    private List<Ball> getFinalListOfBalls() {
        List<Ball> ballsList = new ArrayList<Ball>();

        Color c = Color.WHITE;
        List<Velocity> vel = this.levelInformation.initialBallVelocities();
        for (int i = 0; i < this.levelInformation.numberOfBalls(); i += 1) {
            Ball b = new Ball(397, 540, 6, c);
            b.setVelocity(vel.get(i));
            ballsList.add(b);
        }
        return ballsList;
    }

    /**
     * @param newCounter - new block counter.
     */
    public void setBlockCounter(Counter newCounter) {
        this.counterBlocks = newCounter;
    }

    /**
     * method that return the Collection to his original.
     */
    public void setToDefaultCollection() {

        this.aliensCollection.setCollectionToStart();
    }


    /**
     * set the name of the level.
     */
    public void setLevelName() {
        this.levelInformation = new GameLevel1().setName(" " + this.numberOfLevel);
    }

} // OF CLASS
