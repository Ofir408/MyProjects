import java.util.List;
import java.util.Map;

/**
 * This program is a Sin object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-12
 */

public class Sin extends UnaryExpression implements Expression {

    /**
     * Sin Object have an Expression.
     */
    private Expression sinExpression;

    /**
     * Sin have a constructor accepting an Expression.
     *
     * @param newSinExpression - new expression for the sinus.
     */
    public Sin(final Expression newSinExpression) {
        super(newSinExpression);
    }

    /**
     * @param s - a String.
     */
    public Sin(final String s) {
        super(s);
    }

    /**
     * @param number - a double number.
     */
    public Sin(final double number) {
        super(number);
    }


    @Override
    public double evaluate(Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            System.out.println(" There is at least one variable without"
                    + " his value in the Map, "
                    + "Therefore can't compute this sin ");
            System.exit(1);
        }
        return Math.sin(Math.toRadians(this.getTheExp().evaluate(assignment)));
    }


    /**
     * @param assignment - the map.
     * @return the result of the evaluate, a double number.
     * @throws Exception - here is a variable in sin Expression that
     *                   we dont know this value from assignment map.
     */
    public double evaluate2(Map<String, Double> assignment) throws Exception {
        Expression tempEspression = this.sinExpression;
        List<String> sinVariablesList = this.getVariables();
        while (!assignment.isEmpty() && !sinVariablesList.isEmpty()) {
            int lastIndexInVariablesList = sinVariablesList.size() - 1;
            if (assignment.containsKey(sinVariablesList.get(lastIndexInVariablesList))) {
                Num currentValueToAssign = new Num(assignment.get(
                        lastIndexInVariablesList));
                this.sinExpression = this.assign(sinVariablesList.get(lastIndexInVariablesList),
                        currentValueToAssign);
                // remove this variable from sinVariablesList List:
                sinVariablesList.remove(lastIndexInVariablesList);
            } else {
                // it means there is a variable in sin Expression that
                // we dont know this value from assignment map,
                // therefore, we can't compute this sinExpression.
                throw new Exception("we can't compute this sinExpression,"
                        + "there is a variable in sin Expression that "
                        + " we dont know this value from assignment map");
            }
        }
        // here we have no more variables in this the Expression of the Sin object
        // Therefore compute it with calling to the evaluate func.
        this.sinExpression = tempEspression; // in order do not change the original.
        return this.evaluate();


        // return Math.sin(this.sinExpression.evaluate(assignment) * Math.PI / 180);
        //return Math.sin(this.sinExpression.)
    }

    @Override
    public double evaluate() throws Exception {
        //return Math.sin(Math.toRadians(this.sinExpression.e))
        try {
            double d = (Double.parseDouble(this.getTheExp().
                    toString()));
            //return Math.sin(d * Math.PI / 180);
            return Math.sin(Math.toRadians(d));
        } catch (Exception e) {
            // still need to simplify the expression.
            // therefore do recursive call.
            return this.getTheExp().evaluate();
        }
    }


    @Override
    public String toString() {
        return "sin" + "(" + this.getTheExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // (sin(var))' = cos(var) * (var)'
        Cos c = new Cos(this.getTheExp());
        return new Mult(c, this.getTheExp().differentiate(var));
    }

    @Override
    public Expression simplify() {
        try {
            return new Num(this.evaluate());
        } catch (Exception e) {
            return new Sin(this.getTheExp().simplify());
        }
    }
}
