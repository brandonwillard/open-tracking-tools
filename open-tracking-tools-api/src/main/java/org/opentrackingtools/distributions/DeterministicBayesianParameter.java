package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Random;

public class DeterministicBayesianParameter<T> extends
    AbstractCloneableSerializable
    implements
    BayesianParameter<T, DeterministicDataDistribution<T>, DeterministicDataDistribution<T>> {

  /**
   * 
   */
  private static final long serialVersionUID = -4729393269972527113L;
  protected DeterministicDataDistribution<T> conditionalDistribution;
  protected String name;

  public DeterministicBayesianParameter(
    DeterministicDataDistribution<T> conditionalDistribution,
    String name) {
    super();
    this.conditionalDistribution = conditionalDistribution;
    this.name = name;
  }

  @Override
  public DeterministicBayesianParameter<T> clone() {
    final DeterministicBayesianParameter<T> clone =
        (DeterministicBayesianParameter<T>) super.clone();
    clone.conditionalDistribution =
        this.conditionalDistribution.clone();
    clone.name = this.name;
    return clone;
  }

  @Override
  public DeterministicDataDistribution<T>
      getConditionalDistribution() {
    return this.conditionalDistribution;
  }

  @Override
  public String getName() {
    return this.name;
  }

  @Override
  public DeterministicDataDistribution<T> getParameterPrior() {
    return this.conditionalDistribution;
  }

  @Override
  public T getValue() {
    return this.conditionalDistribution.getElement();
  }

  @Override
  public void setValue(T value) {
    this.conditionalDistribution.set(value, 0);
  }

  @Override
  public void updateConditionalDistribution(Random random) {
  }

}
