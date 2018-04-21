import java.util.Map;
import java.util.TreeMap;

/**
 * This program is a Log object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-13
 */

public class Log extends BinaryExpression implements Expression {

    /**
     * second expression to add.
     */
    private Expression theBase;
    /**
     * first expression to add.
     */
    private Expression expressionToComputeInBase;

    /**
     * @param firstExp  - the first Expression
     * @param secondExp - the second Expression
     */
    public Log(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Log(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression
     * @param str - String
     */
    public Log(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression
     */
    public Log(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Log(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Log(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Log(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Log(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Log(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        assignment.put("e", Math.E);
        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "Log because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return Math.log(this.getSecondExp().evaluate(assignment))
                / Math.log(this.getFirstExp().
                evaluate(assignment));
    }

    @Override
    public double evaluate() throws Exception {
        this.assign("e", new Num(Math.E));
        if (this.getVariables().isEmpty()) {
            // can compute the result
            Map<String, Double> assignment = new TreeMap<String, Double>();
            assignment.put("e", Math.E);
            return this.evaluate(assignment);
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this"
                    + " Log because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "log(" + this.getFirstExp().toString()
                + ", " + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        Const e = new Const(Math.E, "e");
        if (this.getFirstExp().toString() == new Var("e").toString()) {
            return new Mult(new Div(1.0, this.getSecondExp()),
                    this.getSecondExp().differentiate(var));
        }
        Div div = new Div(new Log(e, this.getSecondExp()),
                new Log(e, this.getFirstExp()));
        return div.differentiate(var);
    }

    @Override
    public Expression simplify() {
        Num one = new Num(1);
        try {
            Num n = new Num(this.evaluate());
            return n;
        } catch (Exception e) {
            Expression firstSimplify = this.getFirstExp().simplify();
            Expression secondSimplify = this.getSecondExp().simplify();
            if (this.checkEqualsAlphabetical(firstSimplify.toString(),
                    secondSimplify.toString())) {
                return one;
            } else {
                return new Log(firstSimplify, secondSimplify);
            }
        }
    }
}

