package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.util.CloneableSerializable;

public interface BayesianEstimableParameter<ParameterType, ConditionalType extends ProbabilityFunction<ParameterType>, PriorType extends ComputableDistribution<ParameterType>>
    extends
    BayesianParameter<ParameterType, ConditionalType, PriorType>,
    CloneableSerializable {

  @Override
  public
      BayesianEstimableParameter<ParameterType, ConditionalType, PriorType>
      clone();

}
