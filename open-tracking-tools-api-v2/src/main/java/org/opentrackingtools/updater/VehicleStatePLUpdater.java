package org.opentrackingtools.updater;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.MathUtil;
import gov.sandia.cognition.math.UnivariateStatisticsUtil;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.stat.StatUtils;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.DeterministicDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.OnOffEdgeTransitionEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.model.TransitionProbMatrix;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.primitives.Doubles;

public class VehicleStatePLUpdater<O extends GpsObservation>
    extends AbstractCloneableSerializable implements
    ParticleFilter.Updater<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = 7567157323292175525L;

  protected InferenceGraph inferenceGraph;

  protected O initialObservation;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  public VehicleStatePLUpdater(O obs,
    InferenceGraph inferencedGraph,
    VehicleStateInitialParameters parameters, Random rng) {

    this.initialObservation = obs;
    this.inferenceGraph = inferencedGraph;
    if (rng == null) {
      this.random = new Random();
    } else {
      this.random = rng;
    }
    this.parameters = parameters;

  }

  @Override
  public VehicleStatePLUpdater<O> clone() {
    final VehicleStatePLUpdater<O> clone =
        (VehicleStatePLUpdater<O>) super.clone();
    clone.seed = this.seed;
    clone.inferenceGraph = this.inferenceGraph;
    clone.initialObservation = this.initialObservation;
    clone.parameters = this.parameters;
    clone.random = this.random;
    return clone;
  }

  @Override
  public double computeLogLikelihood(VehicleStateDistribution<O> particle,
    O observation) {
    double logLikelihood = 0d;
    logLikelihood +=
        particle.getMotionStateParam().getConditionalDistribution()
            .getProbabilityFunction()
            .logEvaluate(observation.getProjectedPoint());
    return logLikelihood;
  }

  /**
   * Create vehicle states from the nearby edges.
   */
  @Override
  public DataDistribution<VehicleStateDistribution<O>> createInitialParticles(
    int numParticles) {
    final DataDistribution<VehicleStateDistribution<O>> retDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

    /*
     * Start by creating an off-road vehicle state with which we can obtain the surrounding
     * edges.
     */
    final VehicleStateDistribution<O> nullState =
        VehicleStateDistribution.constructInitialVehicleState(this.parameters, this.inferenceGraph, this.initialObservation, this.random,
            PathEdge.nullPathEdge);
    final MultivariateGaussian initialMotionStateDist =
        nullState.getMotionStateParam().getParameterPrior();
    final Collection<InferenceGraphSegment> edges =
        this.inferenceGraph.getNearbyEdges(initialMotionStateDist,
            initialMotionStateDist.getCovariance());

    for (int i = 0; i < numParticles; i++) {
      /*
       * From the surrounding edges, we create states on those edges.
       */
      final DataDistribution<VehicleStateDistribution<O>> statesOnEdgeDistribution =
          new CountedDataDistribution<VehicleStateDistribution<O>>(true);
      
      final double nullLogLikelihood =
          nullState.getEdgeTransitionParam()
              .getConditionalDistribution()
              .getProbabilityFunction().logEvaluate(InferenceGraphEdge.nullGraphEdge)
              + this.computeLogLikelihood(nullState,
                  this.initialObservation);

      statesOnEdgeDistribution
          .increment(nullState, nullLogLikelihood);

      for (final InferenceGraphSegment segment : edges) {
        
        final PathEdge pathEdge = new PathEdge(segment, 0d, false);
        
        VehicleStateDistribution<O> stateOnEdge = VehicleStateDistribution.constructInitialVehicleState(
            parameters, inferenceGraph, initialObservation, random, pathEdge);

        final double logLikelihood =
            stateOnEdge.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getProbabilityFunction().logEvaluate(pathEdge.getInferenceGraphEdge())
                + this.computeLogLikelihood(stateOnEdge,
                    this.initialObservation);

        statesOnEdgeDistribution
            .increment(stateOnEdge, logLikelihood);
      }

      retDist.increment(statesOnEdgeDistribution.sample(this.random));
    }

    Preconditions.checkState(retDist.getDomainSize() > 0);
    return retDist;
  }

  public InferenceGraph getInferredGraph() {
    return this.inferenceGraph;
  }

  public GpsObservation getInitialObservation() {
    return this.initialObservation;
  }

  public VehicleStateInitialParameters getParameters() {
    return this.parameters;
  }

  public Random getRandom() {
    return this.random;
  }

  public long getSeed() {
    return this.seed;
  }

  public void setRandom(Random random) {
    this.random = random;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

  @Override
  public VehicleStateDistribution<O> update(VehicleStateDistribution<O> state) {
    final VehicleStateDistribution<O> predictedState = new VehicleStateDistribution<O>(state);
    
    /*
     * Predict/project the motion state forward.
     */
    final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
        new MotionStateEstimatorPredictor(state, this.random, this.parameters.getInitialObsFreq());

    predictedState
        .setMotionStateEstimatorPredictor(motionStateEstimatorPredictor);

    final Collection<? extends Path> paths =
        this.inferenceGraph.getPaths(state, state.getObservation());
    
    Preconditions.checkState(paths.contains(Path.nullPath));

    final List<PathStateDistribution> distributions =
        Lists.newArrayList();
    double[] weights = new double[] {};
    double weightsSum = Double.NEGATIVE_INFINITY;
    for (final Path path : paths) {
      final PathStateEstimatorPredictor pathStateEstimatorPredictor =
          new PathStateEstimatorPredictor(state, path);
      
      /*
       * These edge-free prior predictions need to be relative
       * to the assumed starting edge.  Since our paths
       * are both on and off-road, we have to project on when
       * we're off, then predict, and vice versa, when applicable.
       */
      final PathStateDistribution priorPathStateDist = predictedState.getPathStateParam().getParameterPrior();
      final MultivariateGaussian priorMotionState;
      if (path.isNullPath() && priorPathStateDist.getPathState().isOnRoad()) {
        /*
         * Convert to off-road, then predict
         */
        priorMotionState = motionStateEstimatorPredictor
                .createPredictiveDistribution(priorPathStateDist.getGroundDistribution());
      } else if (!path.isNullPath() && !priorPathStateDist.getPathState().isOnRoad()) {
        /*
         * Convert to on-road, then predict
         */
        priorMotionState = PathUtils.getRoadBeliefFromGround(
            predictedState.getPathStateParam().getParameterPrior().getMotionDistribution(), 
            Iterables.getFirst(path.getPathEdges(), null), true);
      } else {
        /*
         * Stay on or off-road, but notice that we use the "local" state (the distance
         * relative to the edge we're on, i.e. the last edge in the path).  If we
         * don't use the local state, then our state distance will increase on each
         * update.
         */
        priorMotionState = 
            predictedState.getPathStateParam().getParameterPrior().getEdgeDistribution();
      }
      final MultivariateGaussian priorPredictiveMotionState = motionStateEstimatorPredictor
                .createPredictiveDistribution(priorMotionState);

      final PathStateMixtureDensityModel pathStateDist =
          pathStateEstimatorPredictor
              .createPredictiveDistribution(priorPredictiveMotionState);
      
      weightsSum = LogMath.add(weightsSum, pathStateDist.getPriorWeightSum());
      distributions.addAll(pathStateDist.getDistributions());
      weights =
          Doubles.concat(weights, pathStateDist.getPriorWeights());
    }
    
    /*
     * Normalize over on/off-road and edge count
     */
    if (paths.size() > 1) {
      for (int i = 0; i < weights.length; i++) {
        /*
         * We need to normalize on-road path weights by the number
         * of edges, otherwise, the number of edges we evaluate will 
         * simply increase the probability of being on a path, which
         * is nonsense.
         */
        if (distributions.get(i).getPathState().isOnRoad()) {
          weights[i] -= Math.log(paths.size() - 1);
        }
        weights[i] -= Math.log(2);
      }
    }
    
    final PathStateMixtureDensityModel predictedPathStateDist =
        new PathStateMixtureDensityModel(
            distributions, weights);
    predictedState.setPathStateParam(SimpleBayesianParameter.create(
        state.getPathStateParam().getParameterPrior().getPathState(),
        predictedPathStateDist, state.getPathStateParam().getParameterPrior()));
    
    return predictedState;
  }

}
