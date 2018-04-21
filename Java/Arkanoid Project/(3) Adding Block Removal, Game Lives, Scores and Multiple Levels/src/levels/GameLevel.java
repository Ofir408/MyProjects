//package threeass;

import java.awt.Color;
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
     * sprites is the name of class: SpriteCollection.
     * that has the collection of the sprites
     */
    private SpriteCollection sprites;

    /**
     * environment is the name of class: GameLevel
     * that has the collection of the Collidables.
     */
    private GameEnvironment environment;

    /**
     * paddle object, for the player in order to move and
     * hit balls in our game.
     */
    private Paddle paddle;

    /**
     * Counter that has details about how much blocks remain,
     * because blocks remove when they have 0 hits - point.
     */
    private Counter counterBlocks;

    /**
     * Counter that has details about how much Balls remain in the game.
     */
    private Counter counterBalls;

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
     * running is true if the game still run,
     * Otherwise - false.
     */
    private boolean running;
    /**
     * Keyboard to get input from the user, like "P" in order
     * to stop the game.
     */
    private KeyboardSensor keyboard;
    /**
     * information about this level.
     */
    private LevelInformation levelInformation;

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
    public GameLevel(final SpriteCollection s, final GameEnvironment g,
                     final Paddle p) {
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
    public GameLevel(final SpriteCollection s, final GameEnvironment g,
                     final Paddle p, final Counter newCounter) {
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
     * @param animationRunner -  AnimationRunner
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
        }

        if (levelInformation.levelName().equals("Wide Easy")) {
            // it means that it's the second level, therefore:
            List<Ball> ballsList = new GameLevel2().getFinalListOfBalls();
            enterBallToGame(ballsList);
        }

        if (levelInformation.levelName().equals("Green 3")) {
            List<Ball> ballsList = new GameLevel3().getFinalListOfBalls();
            enterBallToGame(ballsList);
        }

        if (levelInformation.levelName().equals("Final Four")) {
            List<Ball> ballsList = new GameLevel4().getFinalListOfBalls();
            enterBallToGame(ballsList);
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
        Rectangle r = new Rectangle(new Point(xOfPaddle, 550),
                this.levelInformation.paddleWidth(), 20);
        biuoop.KeyboardSensor keyboardd =
                this.gui.getKeyboardSensor();
        this.keyboard = keyboardd;
        if (this.levelInformation.levelName().equals("Wide Easy")) {
            this.paddle = new Paddle(keyboard, r, this.levelInformation.paddleSpeed(),
                    Color.ORANGE);
            this.paddle.addToGame(this);
            return;
        }
        this.paddle = new Paddle(keyboard, r, this.levelInformation.paddleSpeed(),
                Color.YELLOW);
        this.paddle.addToGame(this);
    }


    /**
     * Initialize a new game: create the Blocks and Ball (and Paddle)
     * and add them to the game.
     */
    public void initialize() {

        BlockRemover bRemover = new BlockRemover(this, new Counter(0));
        this.counterBlocks = bRemover.getCounterRemainBlocks();
        BallRemover ballsRemover = new BallRemover(this, new Counter(0));
        this.counterBalls = ballsRemover.getCounterRemainBalls();
        ScoreTrackingListener sTracking = new ScoreTrackingListener(this.counterScore);
        //this.counterScore = sTracking.getScoreCounter();
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

        // death block to remove balls that hit with him.
        Block deathBlock = new Block(new Rectangle(new Point(
                0, 600), 800, 0.01), 1, Color.YELLOW); // the deathBlock block.
        deathBlock.addToGame(this);
        deathBlock.addHitListener(ballsRemover);

        scoreIndicat.addToGame(this);
        livesInd.addToGame(this);
        NameLevelIndicator nameOfLevel = new NameLevelIndicator(
                this.levelInformation.levelName());
        nameOfLevel.addToGame(this);

    }

    /**
     * is in charge of stopping condition.
     *
     * @return true if the level should stop (because of no more balls,
     * or no more blocks). Otherwise, return false.
     */
    @Override
    public boolean shouldStop() {
        return !this.running;
    }

    /**
     * doOneFrame(DrawSurface) is in charge of the logic.
     *
     * @param d - DrawSurface.
     */
    @Override
    public void doOneFrame(DrawSurface d) {


        this.sprites.drawAllOn(d);
        this.sprites.notifyAllTimePassed();


        Sleeper sleeper = new Sleeper();
        int framesPerSecond = 60;
        int millisecondsPerFrame = 1000 / framesPerSecond;


        while (this.running) {
            d = this.gui.getDrawSurface();
            long startTime = System.currentTimeMillis(); // timing


            this.sprites.drawAllOn(d);

            this.gui.show(d);
            this.sprites.notifyAllTimePassed();


            // timing
            long usedTime = System.currentTimeMillis() - startTime;
            long milliSecondLeftToSleep =
                    millisecondsPerFrame - usedTime;
            if (milliSecondLeftToSleep > 0) {
                sleeper.sleepFor(milliSecondLeftToSleep);
            }

            if (this.counterBlocks.getValue() == 0 || this.counterBalls.getValue() == 0) {
                if (this.counterBlocks.getValue() == 0) {
                    this.counterScore.increase(100);

                } else {
                    // it means no more balls so..
                    if (this.numberOfLives.getValue() > 1) {
                        this.numberOfLives.decreaseOne();
                    } else {
                        // no more life - Lose in the game :(
                        this.runner.run(new YouLose(this.keyboard, this.counterScore.getValue()));
                        gui.close();
                    }

                }
                this.running = false; // stop running.
            }
            if (this.keyboard.isPressed("p")) {
                this.runner.run(new PauseScreen(this.keyboard));
            }
        }
    }

    /**
     * Run the game -- start the animation loop.
     */
    public void playOneTurn() {


        // ********** this.initialize(gui);

        this.createPaddle();
        this.createBalls();


        this.runner = new AnimationRunner(this.gui);
        this.runner.run(new CountdownAnimation(2, 3, this.sprites)); // countdown before turn starts.
        this.running = true;

        // use our runner to run the current animation -- which is one turn of
        // the game.
        //  this.runner = new AnimationRunner(this.gui);
        this.runner.run(this);

    }

    /**
     * run.
     */
    public void run() {


        // this.gui = new GUI("Arkanoid", 800, 600); ////////************/////
        this.initialize();


        while (this.numberOfLives.getValue() > 0) {
            this.playOneTurn();
            this.numberOfLives.decreaseOne();

            if (this.numberOfLives.getValue() == 0 || this.counterBlocks.getValue() == 0) {
                this.gui.close();
            }
        }
    }

    /**
     * @param ballsList - list to enter to the level.
     */
    private void enterBallToGame(List<Ball> ballsList) {

        for (int i = 0; i < ballsList.size(); i += 1) {
            Ball temp = ballsList.get(i);
            temp.setGameEnvironment(this.environment);
            temp.addToGame(this);
            this.counterBalls.increaseOne();
        }
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


} // OF CLASS

