// Pair class, store two values.
class PairObj<K,V> {
    private K first;
    private V second;

    // Constructor.
    public PairObj(K first, V second) { 
        this.first = first;
        this.second = second;
    }

    // get the values.
    public K getKey() { return first; }
    public V getValue() { return second; }
}
