package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.ComputableDistribution;

import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;

public interface BayesianEstimableDistribution<O, V extends ComputableDistribution<O>> {

  public RecursiveBayesianEstimatorPredictor<O, V>
      getBayesianEstimatorPredictor();

}
