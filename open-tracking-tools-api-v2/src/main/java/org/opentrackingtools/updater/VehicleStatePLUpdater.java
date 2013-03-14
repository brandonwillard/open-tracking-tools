package org.opentrackingtools.updater;

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
    final MultivariateGaussian priorMotionState =
        predictedState.getMotionStateParam().getParameterPrior();

    /*
     * Predict/project the motion state forward.
     */
    final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
        new MotionStateEstimatorPredictor(state, this.random, null);

    predictedState
        .setMotionStateEstimatorPredictor(motionStateEstimatorPredictor);

    final MultivariateGaussian priorPredictivePathState =
        motionStateEstimatorPredictor
            .createPredictiveDistribution(priorMotionState);

    final Collection<? extends Path> paths =
        this.inferenceGraph.getPaths(state, state.getObservation());

    final List<PathStateDistribution> distributions =
        Lists.newArrayList();
    double[] weights = new double[] {};
    for (final Path path : paths) {
      final PathStateEstimatorPredictor pathStateEstimatorPredictor =
          new PathStateEstimatorPredictor(state, path);
      final PathStateMixtureDensityModel<PathStateDistribution> pathStateDist =
          pathStateEstimatorPredictor
              .createPredictiveDistribution(priorPredictivePathState);
      distributions.addAll(pathStateDist.getDistributions());
      weights =
          Doubles.concat(weights, pathStateDist.getPriorWeights());
    }
    final PathStateMixtureDensityModel<PathStateDistribution> predictedPathStateDist =
        new PathStateMixtureDensityModel<PathStateDistribution>(
            distributions, weights);
    predictedState.setPathStateParam(SimpleBayesianParameter.create(
        state.getPathStateParam().getParameterPrior().getPathState(),
        predictedPathStateDist, state.getPathStateParam().getParameterPrior()));
    return predictedState;
  }

}
