package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;

import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;

/**
 * This class' main purpose is to yield a distribution over predicted paths,
 * i.e. this is where we obtain paths to evaluate.
 * 
 * @author bwillard
 * 
 * @param <P>
 */
public class PathEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<Path, Vector, PathStateDistribution>,
    IncrementalLearner<PathStateDistribution, DiscreteDistribution<Path>> {

  /**
   * 
   */
  private static final long serialVersionUID = -7682532044773695563L;
  protected VehicleState<GpsObservation> currentState;
  private final GpsObservation obs;

  public PathEstimatorPredictor(
    VehicleState<GpsObservation> currentState, GpsObservation obs) {
    this.currentState = currentState;
    this.obs = obs;
  }

  @Override
  public DiscreteDistribution<Path> createInitialLearnedObject() {
    final DefaultCountedDataDistribution<Path> result =
        new DefaultCountedDataDistribution<Path>(true);
    result.incrementAll(this.currentState.getGraph().getPaths(
        this.currentState, this.obs));
    return result;
  }

  @Override
  public DiscreteDistribution<Path> createPredictiveDistribution(
    PathStateDistribution posterior) {
    return this.createInitialLearnedObject();
  }

  @Override
  public PathStateDistribution learn(Collection<? extends Path> data) {
    return null;
  }

  @Override
  public void update(DiscreteDistribution<Path> target,
    Iterable<? extends PathStateDistribution> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(DiscreteDistribution<Path> target,
    PathStateDistribution data) {
    // TODO Auto-generated method stub

  }

}
