package org.opentrackingtools.distributions;

import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

public class DeterministicBayesianParameter<T> 
  extends AbstractCloneableSerializable 
  implements BayesianParameter<T, DeterministicDataDistribution<T>, DeterministicDataDistribution<T>> {
  
  protected DeterministicDataDistribution<T> conditionalDistribution;
  protected String name;

  public DeterministicBayesianParameter(DeterministicDataDistribution<T> conditionalDistribution, String name) {
    super();
    this.conditionalDistribution = conditionalDistribution;
    this.name = name;
  }

  @Override
  public DeterministicDataDistribution<T> getConditionalDistribution() {
    return conditionalDistribution;
  }

  @Override
  public void setValue(T value) {
    this.conditionalDistribution.set(value, 0);
  }

  @Override
  public T getValue() {
    return this.conditionalDistribution.getElement();
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public DeterministicDataDistribution<T> getParameterPrior() {
    return this.conditionalDistribution;
  }

  @Override
  public void updateConditionalDistribution(Random random) {
  }

}
