package org.opentrackingtools.graph.paths.edges.impl;

import java.util.Collection;

import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.RecursiveBayesianEstimator;

public class SimplePathEdgeEstimator
    implements
    BayesianEstimatorPredictor<ObservationType, ParameterType, PosteriorType>,
    RecursiveBayesianEstimator<ObservationType, ParameterType, BeliefType> {

  public SimplePathEdgeEstimator() {
    // TODO Auto-generated constructor stub
  }

  @Override
  public PosteriorType learn(
    Collection<? extends ObservationType> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public BeliefType createInitialLearnedObject() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void update(BeliefType target, ObservationType data) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void update(BeliefType target,
    Iterable<? extends ObservationType> data) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public ComputableDistribution<ObservationType>
      createPredictiveDistribution(PosteriorType posterior) {
    // TODO Auto-generated method stub
    return null;
  }

}
