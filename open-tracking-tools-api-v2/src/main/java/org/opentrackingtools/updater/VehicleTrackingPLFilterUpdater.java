package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateMixtureDensityModel;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.Collections;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

import com.google.common.collect.Sets;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

public class VehicleTrackingPLFilterUpdater<O extends GpsObservation>
  extends AbstractCloneableSerializable implements ParticleFilter.Updater<O, VehicleState<O>> {

  private static final long serialVersionUID = 7567157323292175525L;

  protected O initialObservation;

  protected InferenceGraph inferenceGraph;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  public VehicleTrackingPLFilterUpdater(O obs,
    InferenceGraph inferencedGraph,
    VehicleStateInitialParameters parameters, Random rng) {

    this.initialObservation = obs;
    this.inferenceGraph = inferencedGraph;
    if (rng == null)
      this.random = new Random();
    else
      this.random = rng;
    this.parameters = parameters;

  }

  @Override
  public VehicleTrackingPLFilterUpdater<O> clone() {
    final VehicleTrackingPLFilterUpdater<O> clone =
        (VehicleTrackingPLFilterUpdater<O>) super.clone();
    clone.seed = seed;
    clone.inferenceGraph = inferenceGraph;
    clone.initialObservation = initialObservation;
    clone.parameters = parameters;
    clone.random = random;
    return clone;
  }

  @Override
  public DataDistribution<VehicleState<O>> createInitialParticles(int numParticles) {
    final DataDistribution<VehicleState<O>> retDist =
        new DefaultCountedDataDistribution<VehicleState<O>>(true);

    for (int i = 0; i < numParticles; i++) {
      final VehicleState<O> state; 
      retDist.increment(state);
    }

    return retDist;
  }

  @Override
  public double computeLogLikelihood(VehicleState<O> particle, O observation) {
    double logLikelihood = 0d;
    logLikelihood +=
        particle.getPathStateParam().getConditionalDistribution().logEvaluate(
            observation.getProjectedPoint());
    return logLikelihood;
  }

  @Override
  public VehicleState<O> update(VehicleState<O> state) {
    final VehicleState<O> predictedState = new VehicleState<O>(state);
    PathStateDistribution priorPathState = predictedState.getPathStateParam().getParameterPrior();
    
    MotionStateEstimatorPredictor motionStateEstimatorPredictor = new MotionStateEstimatorPredictor(state, 
        priorPathState.getPathState().getPath(), this.random);
    PathStateDistribution priorPredictivePathState = motionStateEstimatorPredictor.createPredictiveDistribution(priorPathState);
    
    Collection<? extends Path> paths = this.inferenceGraph.getPaths(state, state.getObservation());
    
    for (Path path : paths) {
      PathStateEstimatorPredictor pathStateEstimatorPredictor = new PathStateEstimatorPredictor(state, path);
      MultivariateMixtureDensityModel<PathStateDistribution> pathStateDist = pathStateEstimatorPredictor.createPredictiveDistribution(pathState);
    }
    return state;
  }

  public InferenceGraph getInferredGraph() {
    return inferenceGraph;
  }

  public GpsObservation getInitialObservation() {
    return initialObservation;
  }

  public VehicleStateInitialParameters getParameters() {
    return parameters;
  }

  public Random getRandom() {
    return random;
  }

  public long getSeed() {
    return seed;
  }

  public void setRandom(Random random) {
    this.random = random;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

}
