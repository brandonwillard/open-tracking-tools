package org.opentrackingtools.updater;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.DefaultBayesianParameter;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultivariateGaussianMeanBayesianEstimator;
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
import org.opentrackingtools.distributions.BayesianEstimableParameter;
import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.DeterministicBayesianParameter;
import org.opentrackingtools.distributions.DeterministicDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransProbabilityFunction;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.SimpleBayesianParameter;
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

  /**
   * Create vehicle states from the nearby edges.
   */
  @Override
  public DataDistribution<VehicleState<O>> createInitialParticles(int numParticles) {
    final DataDistribution<VehicleState<O>> retDist =
        new DefaultCountedDataDistribution<VehicleState<O>>(true);

    for (int i = 0; i < numParticles; i++) {
      /*
       * Start by creating an off-road vehicle state.
       */
      final VehicleState<O> state = constructInitialVehicleState(); 
      MotionStateEstimatorPredictor motionStateEstimatorPredictor = new MotionStateEstimatorPredictor(state, new Path(), this.random);
      PathStateDistribution initialPathState = motionStateEstimatorPredictor.createInitialLearnedObject();
      Collection<InferenceGraphEdge> edges = this.inferenceGraph.getNearbyEdges(
          this.initialObservation.getProjectedPoint(), );
      edges.add(new InferenceGraphEdge());      
      
      retDist.increment(state);
    }

    return retDist;
  }

  /**
   * This is where an initial off-road state is constructed so that 
   * it can be used as a template for the prior initialization of parameters.
   * @return
   */
  protected VehicleState<O> constructInitialVehicleState() {
    
    /*
     * The order of creation is important here.  You must know the
     * dependency between these parameters.
     */
    DeterministicDataDistribution<Matrix> obsCovDistribution = 
        new DeterministicDataDistribution<Matrix>(
            MatrixFactory.getDefault().createDiagonal(this.parameters.getObsCov()));
    BayesianParameter<Matrix, ?, ?> observationCovParam = 
        new DeterministicBayesianParameter<Matrix>(obsCovDistribution, "observationCovariance");
    
    DeterministicDataDistribution<Matrix> onRoadCovDistribution = 
        new DeterministicDataDistribution<Matrix>(
            MatrixFactory.getDefault().createDiagonal(this.parameters.getOnRoadStateCov()));
    BayesianParameter<Matrix, ?, ?> onRoadCovParam =
        new DeterministicBayesianParameter<Matrix>(onRoadCovDistribution, "observationCovariance");
    
    DeterministicDataDistribution<Matrix> offRoadCovDistribution = 
        new DeterministicDataDistribution<Matrix>(
            MatrixFactory.getDefault().createDiagonal(this.parameters.getOnRoadStateCov()));
    BayesianParameter<Matrix, ?, ?> offRoadCovParam =
        new DeterministicBayesianParameter<Matrix>(onRoadCovDistribution, "observationCovariance");
    
    BayesianParameter<Vector, MultivariateGaussian.PDF, MultivariateGaussian>
      motionStateParam = null;
    
    BayesianParameter<? extends InferenceGraphEdge, OnOffEdgeTransProbabilityFunction, OnOffEdgeTransDistribution> 
      edgeTransitionParam = null;
    
    final VehicleState<O> state = new VehicleState<O>(
        this.inferenceGraph,
        this.initialObservation,
        motionStateParam,
        observationCovParam,
        onRoadCovParam,
        offRoadCovParam,
        edgeTransitionParam,
        null);
    
    MotionStateEstimatorPredictor motionStateEstimatorPredictor = new MotionStateEstimatorPredictor(state, 
        new Path(), random);
    
    PathStateDistribution initialDist = motionStateEstimatorPredictor.createInitialLearnedObject();
    
    motionStateParam = new MultivariateGaussianMeanBayesianEstimator.Parameter(initialDist, initialDist);
    
    return state;
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
