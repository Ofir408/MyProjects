import java.util.List;
import java.util.Map;

/**
 * This program is a BaseExpression object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-12.
 */
public abstract class BaseExpression implements Expression {


    /**
     * @param assignment - The map <String, Double>
     * @return true if there is an exception.
     */
    public boolean checkException(Map<String, Double> assignment) {
        List<String> variablesList = this.getVariables();
        int index = 0;
        while (index < variablesList.size()) {
            if (!assignment.containsKey(variablesList.get(index))) {
                return true; // there is exception
            }
            index += 1;
        }
        return false; // no exception
    }

}
