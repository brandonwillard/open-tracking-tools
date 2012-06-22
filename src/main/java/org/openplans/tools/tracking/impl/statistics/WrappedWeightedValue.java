package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.util.DefaultWeightedValue;
import gov.sandia.cognition.util.WeightedValue;

import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;

public class WrappedWeightedValue<T> extends
    DefaultWeightedValue<T> {

  /**
   * 
   */
  private static final long serialVersionUID = -2223108371382713360L;

  @Override
  public String toString() {
    return "WrappedWeightedValue [value=" + value + ", weight="
        + weight + "]";
  }

  public WrappedWeightedValue() {
    super();
  }

  public WrappedWeightedValue(T value, double weight) {
    super(value, weight);
  }

  public WrappedWeightedValue(T value) {
    super(value);
  }

  public WrappedWeightedValue(
    WeightedValue<? extends T> other) {
    super(other);
  }
}
