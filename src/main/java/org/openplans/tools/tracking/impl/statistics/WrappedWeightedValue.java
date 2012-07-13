package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.util.DefaultWeightedValue;
import gov.sandia.cognition.util.WeightedValue;

public class WrappedWeightedValue<T> extends DefaultWeightedValue<T> {

  /**
   * 
   */
  private static final long serialVersionUID = -2223108371382713360L;
  private int count = 0;

  public WrappedWeightedValue() {
    super();
  }

  public WrappedWeightedValue(T value) {
    super(value);
  }

  public WrappedWeightedValue(T value, double weight) {
    super(value, weight);
    this.count++;
  }
  
  public WrappedWeightedValue(T value, double weight, int count) {
    super(value, weight);
    this.count = count;
  }

  public WrappedWeightedValue(WeightedValue<? extends T> other) {
    super(other);
  }

  @Override
  public String toString() {
    return "WrappedWeightedValue [count=" + count + ", value="
        + value + ", weight=" + weight + "]";
  }

  public int getCount() {
    return this.count;
  }
}
