package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.Collections;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.estimators.StandardVehicleStateEstimator;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

public abstract class VehicleTrackingPLFilterUpdater<O extends GpsObservation, V extends VehicleState<O>>
    implements ParticleFilter.Updater<O, V> {

  private static final long serialVersionUID = 7567157323292175525L;

  protected O initialObservation;

  protected InferenceGraph inferenceGraph;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  protected final AbstractRoadTrackingFilter roadFilterGenerator;

  public VehicleTrackingPLFilterUpdater(O obs,
    InferenceGraph inferencedGraph,
    VehicleStateInitialParameters parameters, Random rng)
      throws ClassNotFoundException, SecurityException,
      NoSuchMethodException, IllegalArgumentException,
      InstantiationException, IllegalAccessException,
      InvocationTargetException {

    Class<?> filterType =
        Class.forName(parameters.getRoadFilterTypeName());
    Constructor<?> ctor =
        filterType.getConstructor(GpsObservation.class,
            InferenceGraph.class,
            VehicleStateInitialParameters.class, Random.class);

    this.roadFilterGenerator =
        (AbstractRoadTrackingFilter) ctor.newInstance(obs,
            inferencedGraph, parameters, rng);
    this.initialObservation = obs;
    this.inferenceGraph = inferencedGraph;
    if (rng == null)
      this.random = new Random();
    else
      this.random = rng;
    this.parameters = parameters;

  }

  @Override
  public VehicleTrackingPLFilterUpdater clone() {
    try {
      final VehicleTrackingPLFilterUpdater clone =
          (VehicleTrackingPLFilterUpdater) super.clone();
      clone.seed = seed;
      clone.inferenceGraph = inferenceGraph;
      clone.initialObservation = initialObservation;
      clone.parameters = parameters;
      clone.random = random;
      return clone;
    } catch (final CloneNotSupportedException e) {
      e.printStackTrace();
    }
    return null;
  }

  @Override
  public DataDistribution<V> createInitialParticles(int numParticles) {
    final DataDistribution<V> retDist =
        new DefaultCountedDataDistribution<V>(true);

    RecursiveBayesianEstimatorPredictor<O, V> estimator =
        new StandardVehicleStateEstimator<O, V>(
            this.initialObservation);
    for (int i = 0; i < numParticles; i++) {
      final V state = estimator.createInitialLearnedObject();
      retDist.increment(state);
    }

    return retDist;
  }

  @Override
  public double computeLogLikelihood(V particle, O observation) {
    double logLikelihood = 0d;
    logLikelihood +=
        particle.getBelief().logLikelihood(
            observation.getProjectedPoint(),
            particle.getMovementFilter());
    return logLikelihood;
  }

  @Override
  public V update(V state) {
    state.getBayesianEstimatorPredictor().update(state,
        this.initialObservation);
    return state;
  }

  protected AbstractRoadTrackingFilter createRoadTrackingFilter() {
    return roadFilterGenerator.clone();
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
