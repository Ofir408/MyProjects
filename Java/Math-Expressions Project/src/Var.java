import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * This program is a Var object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-11
 */

public class Var implements Expression {

    /**
     * the name of the variable - a string.
     */
    private String varName;

    /**
     * constructor of Var object.
     *
     * @param nameOfVar - the name of the new variable.
     */
    public Var(final String nameOfVar) {
        this.varName = nameOfVar;
    }


    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        // check if the current (this) variable apper in this Map,
        // if not then throw execption.
        if (assignment.get(this.varName) != null) {
            return assignment.get(this.varName);
        } else {
            throw new Exception(" The variable " + this.varName
                    + " does not appered"
                    + " in assigment.");
        }
    }

    @Override
    public double evaluate() throws Exception {
        // no needed.
        return 0; // do not use in this function.
    }

    @Override
    public List<String> getVariables() {
        List<String> l = new ArrayList<String>();
        l.add(this.varName);
        return l;
    }

    @Override
    public Expression assign(String var, Expression expression) {
        return expression;
    }

    @Override
    public String toString() {
        return this.varName;
    }


    @Override
    public Expression differentiate(String var) {
        // check if this is constant number, if yes return 0
        if (!this.varName.equals(var)) {
            // it means it is a constant number
            return new Num(0.0);
        } else {
            // it a variable
            return new Num(1.0);
        }
    }

    @Override
    public Expression simplify() {
        return this;
    }
}
