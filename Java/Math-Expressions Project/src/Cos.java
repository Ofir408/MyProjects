import java.util.Map;

/**
 * This program is a Cos object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-13
 */

public class Cos extends UnaryExpression implements Expression {

    /**
     * Cos Object have an Expression.
     */
    private Expression cosExpression;

    /**
     * Cos have a constructor accepting an Expression.
     *
     * @param newSinExpression - new expression for the cos.
     */
    public Cos(final Expression newSinExpression) {
        super(newSinExpression);
    }

    /**
     * @param s - a String.
     */
    public Cos(final String s) {
        super(s);
    }

    /**
     * @param number - a double number.
     */
    public Cos(final double number) {
        super(number);
    }

    @Override
    public double evaluate(Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            System.out.println(" There is variable without"
                    + " his value in the Map, "
                    + "Therefore can't compute this cos ");
            System.exit(1);
        }
        return Math.cos(Math.toRadians(this.getTheExp().
                evaluate(assignment)));
    }

    @Override
    public double evaluate() throws Exception {
        try {
            double d = (Double.parseDouble(this.getTheExp().
                    toString()));
            return Math.cos(Math.toRadians(d));
        } catch (Exception e) {
            // still need to simplify the expression.
            // therefore do recursive call.
            return this.getTheExp().evaluate();
        }
    }


    @Override
    public String toString() {
        return "cos" + "(" + this.getTheExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // (cos(var))' = -sin(var) * (var)'
        Sin s = new Sin(this.getTheExp());
        //return new Mult(new Neg(s), this.getTheExp().differentiate(var));
        return new Neg(new Mult(s, this.getTheExp().differentiate(var)));
    }

    @Override
    public Expression simplify() {
        try {
            return new Num(this.evaluate());
        } catch (Exception e) {
            return new Cos(this.getTheExp().simplify());
        }
    }


}
