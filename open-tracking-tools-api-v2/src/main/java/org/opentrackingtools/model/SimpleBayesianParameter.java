package org.opentrackingtools.model;

import java.util.Random;

import gov.sandia.cognition.statistics.ClosedFormDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.CloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

public class SimpleBayesianParameter<ParameterType, ConditionalType extends ClosedFormDistribution<?>, PriorType extends ClosedFormDistribution<ParameterType>>
  extends AbstractCloneableSerializable
  implements BayesianParameter<ParameterType, ConditionalType, PriorType> {

  protected ConditionalType conditional;
  protected PriorType prior;
  protected ParameterType value;
  protected String name = null;
  
  @Override
  public SimpleBayesianParameter<ParameterType, ConditionalType, PriorType> clone() {
    SimpleBayesianParameter<ParameterType, ConditionalType, PriorType> clone = 
        (SimpleBayesianParameter<ParameterType, ConditionalType, PriorType>) super.clone();
    clone.conditional = ObjectUtil.cloneSmart(this.conditional);
    clone.prior = ObjectUtil.cloneSmart(this.prior);
    clone.value = ObjectUtil.cloneSmart(this.value);
    return clone;
  }
  
  public SimpleBayesianParameter(SimpleBayesianParameter<ParameterType, ConditionalType, PriorType> other) {
    super();
    this.conditional = other.conditional;
    this.prior = other.prior;
    this.value = other.value;
  }
  
  public static <Par, C extends ClosedFormDistribution<?>, P extends ClosedFormDistribution<Par>> 
    SimpleBayesianParameter<Par, C, P> create(Par value, C conditional, P prior) {
    return new SimpleBayesianParameter<Par, C, P>(conditional, prior, value);
  }

  public SimpleBayesianParameter(ConditionalType conditional,
    PriorType prior, ParameterType value) {
    super();
    this.conditional = conditional;
    this.prior = prior;
    this.value = value;
  }

  @Override
  public ConditionalType getConditionalDistribution() {
    return this.conditional;
  }

  @Override
  public void setValue(ParameterType value) {
    this.value = value;
  }

  @Override
  public ParameterType getValue() {
    return this.value;
  }

  @Override
  public String getName() {
    return this.name;
  }

  @Override
  public PriorType getParameterPrior() {
    return this.prior;
  }

  @Override
  public void updateConditionalDistribution(Random random) {
  }

  public void setParameterPrior(
    PriorType prior) {
    this.prior = prior;
  }

}
