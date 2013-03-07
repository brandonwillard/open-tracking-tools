package org.opentrackingtools.distributions;

import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;

import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.util.CloneableSerializable;

public interface BayesianEstimableDistribution<O, V extends ComputableDistribution<O>> {

  public RecursiveBayesianEstimatorPredictor<O, V>
      getBayesianEstimatorPredictor();
  
}
