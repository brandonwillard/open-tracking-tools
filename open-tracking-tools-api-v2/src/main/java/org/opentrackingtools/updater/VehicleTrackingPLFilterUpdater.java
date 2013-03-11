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
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.model.TransitionProbMatrix;

import com.beust.jcommander.internal.Lists;
import com.google.common.primitives.Doubles;

public class VehicleTrackingPLFilterUpdater<O extends GpsObservation>
    extends AbstractCloneableSerializable implements
    ParticleFilter.Updater<O, VehicleState<O>> {

  private static final long serialVersionUID = 7567157323292175525L;

  protected InferenceGraph inferenceGraph;

  protected O initialObservation;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  public VehicleTrackingPLFilterUpdater(O obs,
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
  public VehicleTrackingPLFilterUpdater<O> clone() {
    final VehicleTrackingPLFilterUpdater<O> clone =
        (VehicleTrackingPLFilterUpdater<O>) super.clone();
    clone.seed = this.seed;
    clone.inferenceGraph = this.inferenceGraph;
    clone.initialObservation = this.initialObservation;
    clone.parameters = this.parameters;
    clone.random = this.random;
    return clone;
  }

  @Override
  public double computeLogLikelihood(VehicleState<O> particle,
    O observation) {
    double logLikelihood = 0d;
    logLikelihood +=
        particle.getMotionStateParam().getConditionalDistribution()
            .getProbabilityFunction()
            .logEvaluate(observation.getProjectedPoint());
    return logLikelihood;
  }

  /**
   * This is where an initial off-road state is constructed so that it can be
   * used as a template for the prior initialization of parameters.
   * 
   * @return
   */
  protected VehicleState<O> constructInitialVehicleState() {

    /*
     * The order of creation is important here.  You must know the
     * dependency between these parameters.
     */
    final DeterministicDataDistribution<Matrix> obsCovDistribution =
        new DeterministicDataDistribution<Matrix>(MatrixFactory
            .getDefault().createDiagonal(this.parameters.getObsCov()));
    final SimpleBayesianParameter<Matrix, ?, ?> observationCovParam =
        SimpleBayesianParameter.create(obsCovDistribution.getElement(), obsCovDistribution, 
           obsCovDistribution);

    final DeterministicDataDistribution<Matrix> onRoadCovDistribution =
        new DeterministicDataDistribution<Matrix>(MatrixFactory
            .getDefault().createDiagonal(
                this.parameters.getOnRoadStateCov()));
    final SimpleBayesianParameter<Matrix, ?, ?> onRoadCovParam =
        SimpleBayesianParameter.create(onRoadCovDistribution.getElement(),
            onRoadCovDistribution, onRoadCovDistribution);

    final DeterministicDataDistribution<Matrix> offRoadCovDistribution =
        new DeterministicDataDistribution<Matrix>(MatrixFactory
            .getDefault().createDiagonal(
                this.parameters.getOffRoadStateCov()));
    final SimpleBayesianParameter<Matrix, ?, ?> offRoadCovParam =
        SimpleBayesianParameter.create(offRoadCovDistribution.getElement(),
            offRoadCovDistribution, offRoadCovDistribution);

    final VehicleState<O> state =
        new VehicleState<O>(this.inferenceGraph,
            this.initialObservation, null, null, observationCovParam,
            onRoadCovParam, offRoadCovParam, null, null);

    final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
        new MotionStateEstimatorPredictor(state, this.random,
            (double) this.parameters.getInitialObsFreq());

    final MultivariateGaussian initialMotionStateDist =
        motionStateEstimatorPredictor.createInitialLearnedObject();

    final InferenceGraphEdge nullGraphEdge = new InferenceGraphEdge();
    final MultivariateGaussian initialObservationState =
        motionStateEstimatorPredictor.getObservationDistribution(
            initialMotionStateDist, nullGraphEdge);

    final SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam =
        SimpleBayesianParameter.create( initialObservationState.getMean(),
            initialObservationState, initialMotionStateDist);
    state.setMotionStateParam(motionStateParam);

    final Path path = new Path();
    final PathStateDistribution initialPathStateDist =
        new PathStateDistribution(path, initialMotionStateDist);
    final SimpleBayesianParameter<PathState, PathStateMixtureDensityModel<PathStateDistribution>, PathStateDistribution> pathStateParam =
        SimpleBayesianParameter.create(
            initialPathStateDist.getPathState(),
            new PathStateMixtureDensityModel<PathStateDistribution>(
                Collections.singleton(initialPathStateDist)), initialPathStateDist);
    state.setPathStateParam(pathStateParam);

    final InferenceGraphEdge nullEdge = new InferenceGraphEdge();
    final OnOffEdgeTransPriorDistribution initialPriorTransDist =
        new OnOffEdgeTransPriorDistribution(this.parameters.getOnTransitionProbs(), 
            this.parameters.getOffTransitionProbs());
    final TransitionProbMatrix transitionProbMatrix = new TransitionProbMatrix(
            initialPriorTransDist.getEdgeMotionTransProbPrior().getMean(), 
            initialPriorTransDist.getFreeMotionTransProbPrior().getMean());
    final OnOffEdgeTransDistribution initialTransDist =
        new OnOffEdgeTransDistribution(state, nullEdge,
            initialPriorTransDist.getEdgeMotionTransProbPrior().getMean(), 
            initialPriorTransDist.getFreeMotionTransProbPrior().getMean());

    final SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> edgeTransitionParam =
        SimpleBayesianParameter.create(transitionProbMatrix,
            initialTransDist, initialPriorTransDist);
    state.setEdgeTransitionParam(edgeTransitionParam);

    return state;
  }

  /**
   * Create vehicle states from the nearby edges.
   */
  @Override
  public DataDistribution<VehicleState<O>> createInitialParticles(
    int numParticles) {
    final DataDistribution<VehicleState<O>> retDist =
        new CountedDataDistribution<VehicleState<O>>(true);

    /*
     * Start by creating an off-road vehicle state with which we can obtain the surrounding
     * edges.
     */
    final InferenceGraphEdge nullEdge = new InferenceGraphEdge();
    final VehicleState<O> nullState =
        this.constructInitialVehicleState();
    final MultivariateGaussian initialMotionStateDist =
        nullState.getMotionStateParam().getParameterPrior();
    final Collection<InferenceGraphEdge> edges =
        this.inferenceGraph.getNearbyEdges(initialMotionStateDist,
            initialMotionStateDist.getCovariance());
    edges.add(nullEdge);

    for (int i = 0; i < numParticles; i++) {
      /*
       * From the surrounding edges, we create states on those edges.
       */
      final DataDistribution<VehicleState<O>> statesOnEdgeDistribution =
          new CountedDataDistribution<VehicleState<O>>(true);

      for (final InferenceGraphEdge edge : edges) {
        final SimpleBayesianParameter obsCovParam = nullState.getObservationCovarianceParam().clone();
        final SimpleBayesianParameter onRoadCovParam = nullState.getOnRoadModelCovarianceParam().clone();
        final SimpleBayesianParameter offRoadCovParam = nullState.getOffRoadModelCovarianceParam().clone();
        final SimpleBayesianParameter edgeTransPram = 
            nullState.getEdgeTransitionParam().clone();

        final VehicleState<O> stateOnEdge =
            new VehicleState<O>(this.inferenceGraph,
                this.initialObservation, null, null, obsCovParam,
                onRoadCovParam, offRoadCovParam, edgeTransPram, null);

        final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
            new MotionStateEstimatorPredictor(nullState, this.random,
                (double) this.parameters.getInitialObsFreq());

        final MultivariateGaussian edgeMotionStateDist =
            motionStateEstimatorPredictor
                .createInitialLearnedObject();

        final MultivariateGaussian initialObservationState =
            motionStateEstimatorPredictor.getObservationDistribution(
                edgeMotionStateDist, edge);

        final SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam =
            SimpleBayesianParameter.create(initialObservationState.getMean(),
                initialObservationState, edgeMotionStateDist);

        stateOnEdge.setMotionStateParam(motionStateParam);

        final PathEdge<?> pathEdge =
            new PathEdge<InferenceGraphEdge>(edge, 0d, false);
        final Path path = new Path(pathEdge);
        final PathStateDistribution initialPathStateDist =
            new PathStateDistribution(path, initialMotionStateDist);
        final PathStateMixtureDensityModel<PathStateDistribution> pathStateMixture = 
            new PathStateMixtureDensityModel<PathStateDistribution>(Collections.singleton(initialPathStateDist));
        final SimpleBayesianParameter<PathState, PathStateMixtureDensityModel<PathStateDistribution>, PathStateDistribution> edgeStateParam =
            SimpleBayesianParameter.create(initialPathStateDist.getPathState(),
                pathStateMixture, initialPathStateDist);

        stateOnEdge.setPathStateParam(edgeStateParam);

        final double logLikelihood =
            stateOnEdge.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getProbabilityFunction().logEvaluate(edge)
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
  public VehicleState<O> update(VehicleState<O> state) {
    final VehicleState<O> predictedState = new VehicleState<O>(state);
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
