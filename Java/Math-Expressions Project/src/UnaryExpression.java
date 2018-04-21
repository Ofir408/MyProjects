import java.util.List;

/**
 * This program is a UnaryExpression object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-13
 */
public abstract class UnaryExpression extends BaseExpression implements Expression {

    /**
     * UnaryExpression Object has an Expression.
     */
    private Expression theExpression;

    /**
     * UnaryExpression has a constructor accepting an Expression.
     *
     * @param newUnaryExpression - new expression for the UnaryExpression.
     */
    public UnaryExpression(final Expression newUnaryExpression) {
        this.theExpression = newUnaryExpression;
    }

    /**
     * @param newString - get new String.
     */
    public UnaryExpression(final String newString) {
        this.theExpression = new Var(newString);
    }

    /**
     * @param doubleNumber - get new double number.
     */
    public UnaryExpression(final double doubleNumber) {
        this.theExpression = new Num(doubleNumber);
    }


    /**
     * @return the Expression.
     */
    public Expression getTheExp() {
        return this.theExpression;
    }

    /**
     * @return list of the Variables.
     */
    @Override
    public List<String> getVariables() {
        List<String> variablesList =
                this.getTheExp().getVariables();
        return variablesList;
    }

    @Override
    public Expression assign(final String var,
                             final Expression expression) {
        if (this.getTheExp().getVariables().indexOf(var)
                != -1) {
            this.theExpression =
                    this.getTheExp().assign(var,
                            expression);
        }
        Expression e = this;
        return e;
    }
}
