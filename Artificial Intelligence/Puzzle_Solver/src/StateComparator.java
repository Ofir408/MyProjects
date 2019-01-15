import java.util.Comparator;

public class StateComparator<T> implements Comparator<T> {

	@Override
	// <0 if first less than second.
	// >0 if the first bigger than second
	public int compare(T arg0, T arg1) {
		AStarState first = (AStarState)arg0;
		AStarState second = (AStarState)arg1;
		
		double funcValOfFirst = first.getgFuncVal() + first.gethFunc();
		double funcValOfSecond = second.getgFuncVal() + second.gethFunc();
		int rv = (int)Math.round(funcValOfFirst - funcValOfSecond);
		if (rv == 0)
		rv = first.getCreationTime()  - second.getCreationTime();

		if (rv == 0) {
			char firstLastMove = first.getPath().charAt(first.getPath().length() - 1);
			char secLastMove =  second.getPath().charAt(second.getPath().length() - 1);
			rv = compareHelper(firstLastMove, secLastMove);
		}
		return rv;
	}
	
	// helper function that compares two possible steps.
	private int compareHelper(char first, char second) {
		int firstPriority = getStepPriority(first);
		int secPriority = getStepPriority(second);
		return firstPriority - secPriority;
	}
	
	// get priority of c,
	private int getStepPriority(char c) {
		if (c == 'U')
			return 1;
		if (c == 'D')
			return 2; 
		if (c == 'L')
			return 3; 
		return 4;
	}
}
