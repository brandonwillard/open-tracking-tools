package org.opentrackingtools.estimators;

import java.util.Collection;
import java.util.List;

import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.PathEdgeDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;

import com.google.common.collect.Iterables;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.RecursiveBayesianEstimator;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

/**
 * This class' main purpose is to yield a distribution over
 * predicted paths, i.e. this is where we obtain paths to evaluate.
 * 
 * @author bwillard
 *
 * @param <P>
 */
public class PathEstimatorPredictor extends
    AbstractCloneableSerializable implements
    BayesianEstimatorPredictor<Path, Vector, PathStateDistribution>, 
    IncrementalLearner<PathStateDistribution, DiscreteDistribution<Path>> {

  protected VehicleState<GpsObservation> currentState;
  private GpsObservation obs;
  
  public PathEstimatorPredictor(VehicleState<GpsObservation> currentState, GpsObservation obs) {
    this.currentState = currentState;
    this.obs = obs;
  }

  @Override
  public DiscreteDistribution<Path> createPredictiveDistribution(
    PathStateDistribution posterior) {
    return this.createInitialLearnedObject();
  }

  @Override
  public DiscreteDistribution<Path> createInitialLearnedObject() {
    DefaultCountedDataDistribution<Path> result = new DefaultCountedDataDistribution<Path>(true);
    result.incrementAll(this.currentState.getGraph().getPaths(this.currentState, obs));
    return result;
  }

  @Override
  public PathStateDistribution learn(Collection<? extends Path> data) {
    return null;
  }

  @Override
  public void update(DiscreteDistribution<Path> target,
    PathStateDistribution data) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void update(DiscreteDistribution<Path> target,
    Iterable<? extends PathStateDistribution> data) {
    // TODO Auto-generated method stub
    
  }


}
