import java.util.List;
import java.util.Map;

/**
 * This program is a Div object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-13
 */

public class Div extends BinaryExpression implements Expression {

    /**
     * first expression to add.
     */
    private Expression firstExpression;

    /**
     * second expression to add.
     */
    private Expression secondExpression;

    /**
     * @param firstExp  - the first Expression to add
     * @param secondExp - the second Expression to add
     */
    public Div(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Div(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression to add.
     * @param str - String
     */
    public Div(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression to add.
     */
    public Div(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Div(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Div(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Div(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Div(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Div(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "Div because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return this.getFirstExp().evaluate(assignment)
                / this.getSecondExp().
                evaluate(assignment);
    }

    @Override
    public double evaluate() throws Exception {
        if (this.getVariables().isEmpty()) {
            // can compute the result
            return this.getFirstExp().evaluate()
                    / this.getSecondExp().evaluate();
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this"
                    + " Div because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "(" + this.getFirstExp().toString() + " / "
                + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // ( f(x) / (g(x)) )' = (f(x)' * g(x) - g(x)' * f(x) ) / (g(x) ^ 2).
        Expression f = this.getFirstExp();
        Expression g = this.getSecondExp();
        Mult m1 = new Mult(f.differentiate(var), g);
        Mult m2 = new Mult(f, g.differentiate(var));
        Minus moneResult = new Minus(m1, m2);
        Pow denominatorResult = new Pow(g, 2);
        return new Div(moneResult, denominatorResult);
    }

    @Override
    public Expression simplify() {
        try {

            Num n = new Num(this.evaluate());
            return n;
        } catch (Exception e) {
            Expression firstSimplify = this.getFirstExp().simplify();
            Expression secondSimplify = this.getSecondExp().simplify();
            // check x / x
            String first = firstSimplify.toString();
            String second = secondSimplify.toString();
            if (this.checkEqualsAlphabetical(first, second)) {
                return new Num(1); // x/x = 1
            }
            // check x/1
            if (second.equals(new Num(1).toString())) {
                return firstSimplify; // x/1 = x
            }
            // for the bonus
            if (((firstSimplify instanceof Div || firstSimplify instanceof Num)
                    && (secondSimplify instanceof Div
                    || secondSimplify instanceof Num))) {
                return this.commonFactor();
            }
            return this.divideVar(firstSimplify, secondSimplify);
        }
    }

    /**
     * @return the commonFactor Expression.
     */
    public Expression commonFactor() {
        //if (this instanceof Div) {
        System.out.println("here!!");
        List<String> firstVars = this.getFirstExp().getVariables();
        List<String> secondVars = this.getSecondExp().getVariables();
        if (firstVars.containsAll(secondVars) && secondVars.containsAll(firstVars)) {
            double number = this.gcd(this.getNumberFromExp(this.getFirstExp()),
                    this.getNumberFromExp(this.getSecondExp()));
            Mult m1 = new Mult(number, firstVars.get(0));
            System.out.println("first after sim"
                    + new Div(this.getFirstExp(), m1).simplify());
            Expression resultOne = new Div(this.getFirstExp(), m1).simplify();
            Expression resultTwo = new Div(this.getSecondExp(), m1).simplify();
            Mult finalResult = new Mult(m1, new Div(resultOne, resultTwo));
            return (finalResult.simplify());
        }
        //}
        return new Plus(this.getFirstExp(), this.getSecondExp());
    }


    /**
     * @param exp - the expression for this checking.
     * @return the number from Exp,
     * For example for 3x - return 3.
     */
    private double getNumberFromExp(Expression exp) {
        double number;
        if (exp instanceof Num) {
            Num expIsNum = (Num) exp;
            number = expIsNum.getNumebr();
        } else {
            // Mult expression, according our if
            // Statement that calls to this function.
            Div expIsMult = (Div) exp;
            // the number appears from left (firstExp).
            Num num = (Num) expIsMult.getFirstExp();
            number = num.getNumebr();
        }
        return number;
    }

    /**
     * @param first  - the first Expression.
     * @param second - the second Expression.
     * @return the Expression after divide the common variables.
     */
    private Expression divideVar(Expression first, Expression second) {
        if (first instanceof Mult || second instanceof Mult) {
            try {
                Mult f = (Mult) first;
                Mult s = new Mult(new Num(1), second);
                if (second instanceof Mult) {
                    s = (Mult) (second);
                }
                if (second.simplify() instanceof Var) {
                    if (this.checkEqualsAlphabetical(f.getSecondExp().toString(),
                            s.getSecondExp().toString())) {
                        return new Div(f.getFirstExp(), s.getFirstExp());
                    }
                }
                if (this.checkEqualsAlphabetical(f.getSecondExp().toString(),
                        s.getSecondExp().toString())) {
                    return new Div(f.getFirstExp(), s.getFirstExp());
                }
            } catch (Exception e) {
                return new Div(first, second);
            }
        }
        return new Div(first, second);
    }
}

