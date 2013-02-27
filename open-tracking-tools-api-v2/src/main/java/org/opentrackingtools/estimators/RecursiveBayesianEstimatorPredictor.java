package org.opentrackingtools.estimators;

import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.RecursiveBayesianEstimator;

public interface RecursiveBayesianEstimatorPredictor<O, V extends ComputableDistribution<O>>
    extends BayesianEstimatorPredictor<O, O, V>,
    RecursiveBayesianEstimator<O, O, V> {

  @Override
  public void update(V state, O data);

  @Override
  public V createPredictiveDistribution(V posterior);

  @Override
  public V createInitialLearnedObject();

}
