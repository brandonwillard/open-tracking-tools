package org.opentrackingtools.updater;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.List;
import java.util.Random;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.PathUtils;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.primitives.Doubles;

public class VehicleStatePLUpdater<O extends GpsObservation, G extends InferenceGraph>
    extends AbstractCloneableSerializable implements
    ParticleFilter.Updater<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = 7567157323292175525L;

  protected G inferenceGraph;

  protected O initialObservation;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  protected VehicleStateDistributionFactory<O, G> vehicleStateFactory;

  public VehicleStatePLUpdater(O obs, G inferencedGraph,
    VehicleStateDistributionFactory<O, G> vehicleStateFactory,
    VehicleStateInitialParameters parameters, Random rng) {
    this.vehicleStateFactory = vehicleStateFactory;
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
  public VehicleStatePLUpdater<O, G> clone() {
    final VehicleStatePLUpdater<O, G> clone =
        (VehicleStatePLUpdater<O, G>) super.clone();
    clone.seed = this.seed;
    clone.inferenceGraph = this.inferenceGraph;
    clone.initialObservation = this.initialObservation;
    clone.parameters = this.parameters;
    clone.random = this.random;
    return clone;
  }

  @Override
  public double computeLogLikelihood(
    VehicleStateDistribution<O> particle, O observation) {
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
  public DataDistribution<VehicleStateDistribution<O>>
      createInitialParticles(int numParticles) {
    final DataDistribution<VehicleStateDistribution<O>> retDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

    /*
     * Start by creating an off-road vehicle state with which we can obtain the surrounding
     * edges.
     */
    final VehicleStateDistribution<O> nullState =
        this.vehicleStateFactory.createInitialVehicleState(
            this.parameters, this.inferenceGraph,
            this.initialObservation, this.random,
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
          new CountedDataDistribution<VehicleStateDistribution<O>>(
              true);

      final double nullLogLikelihood =
          nullState.getEdgeTransitionParam()
              .getConditionalDistribution().getProbabilityFunction()
              .logEvaluate(InferenceGraphEdge.nullGraphEdge)
              + this.computeLogLikelihood(nullState,
                  this.initialObservation);

      statesOnEdgeDistribution
          .increment(nullState, nullLogLikelihood);

      for (final InferenceGraphSegment segment : edges) {

        final PathEdge pathEdge = new PathEdge(segment, 0d, false);

        final VehicleStateDistribution<O> stateOnEdge =
            this.vehicleStateFactory.createInitialVehicleState(
                this.parameters, this.inferenceGraph,
                this.initialObservation, this.random, pathEdge);

        final double logLikelihood =
            stateOnEdge.getEdgeTransitionParam()
                .getConditionalDistribution()
                .getProbabilityFunction()
                .logEvaluate(pathEdge.getInferenceGraphEdge())
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

  public G getInferenceGraph() {
    return this.inferenceGraph;
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

  public VehicleStateDistributionFactory<O, G>
      getVehicleStateFactory() {
    return this.vehicleStateFactory;
  }

  public void setInferenceGraph(G inferenceGraph) {
    this.inferenceGraph = inferenceGraph;
  }

  public void setInitialObservation(O initialObservation) {
    this.initialObservation = initialObservation;
  }

  public void setParameters(VehicleStateInitialParameters parameters) {
    this.parameters = parameters;
  }

  public void setRandom(Random random) {
    this.random = random;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

  public void setVehicleStateFactory(
    VehicleStateDistributionFactory<O, G> vehicleStateFactory) {
    this.vehicleStateFactory = vehicleStateFactory;
  }

  @Override
  public VehicleStateDistribution<O> update(
    VehicleStateDistribution<O> state) {
    final VehicleStateDistribution<O> predictedState = state.clone();

    /*
     * Predict/project the motion state forward.
     */
    final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
        new MotionStateEstimatorPredictor(state, this.random,
            this.parameters.getInitialObsFreq());

    predictedState
        .setMotionStateEstimatorPredictor(motionStateEstimatorPredictor);

    final Collection<? extends Path> paths =
        this.inferenceGraph.getPaths(predictedState,
            predictedState.getObservation());

    Preconditions.checkState(paths.contains(Path.nullPath));

    final List<PathStateDistribution> distributions =
        Lists.newArrayList();
    int numberOfNonZeroPaths = 0;
    int numberOfOffRoadPaths = 0;
    double[] weights = new double[] {};
    double weightsSum = Double.NEGATIVE_INFINITY;
    for (final Path path : paths) {
      final PathStateEstimatorPredictor pathStateEstimatorPredictor =
          new PathStateEstimatorPredictor(state, path,
              this.parameters.getInitialObsFreq());

      /*
       * These edge-free prior predictions need to be relative
       * to the assumed starting edge.  Since our paths
       * are both on and off-road, we have to project on when
       * we're off, then predict, and vice versa, when applicable.
       */
      final PathStateDistribution priorPathStateDist =
          predictedState.getPathStateParam().getParameterPrior();
      final MultivariateGaussian priorPredictiveMotionState;
      if (path.isNullPath()
          && priorPathStateDist.getPathState().isOnRoad()) {
        /*
         * Convert to off-road, then predict
         */
        final MultivariateGaussian priorMotionState =
            priorPathStateDist.getMotionDistribution().clone();
        PathUtils.convertToGroundBelief(priorMotionState,
            priorPathStateDist.getPathState().getEdge(), true, false,
            true);
        priorPredictiveMotionState =
            motionStateEstimatorPredictor
                .createPredictiveDistribution(priorMotionState);
      } else if (!path.isNullPath()
          && !priorPathStateDist.getPathState().isOnRoad()) {
        final MultivariateGaussian priorMotionState =
            motionStateEstimatorPredictor
                .createPredictiveDistribution(priorPathStateDist
                    .getMotionDistribution());

        priorPredictiveMotionState =
            PathUtils.getRoadBeliefFromGround(priorMotionState,
                Iterables.getFirst(path.getPathEdges(), null), true,
                priorPathStateDist.getMotionDistribution().getMean(),
                this.parameters.getInitialObsFreq());
        /*
         * Not allowing backward movement...
         */
        if (priorPredictiveMotionState.getMean().getElement(1) <= 0d) {
          continue;
        }
      } else {
        /*
         * Stay on or off-road, but notice that we use the "local" state (the distance
         * relative to the edge we're on, i.e. the last edge in the path).  If we
         * don't use the local state, then our state distance will increase on each
         * update.
         */
        final MultivariateGaussian priorMotionState =
            predictedState.getPathStateParam().getParameterPrior()
                .getEdgeDistribution();
        priorPredictiveMotionState =
            motionStateEstimatorPredictor
                .createPredictiveDistribution(priorMotionState);
      }

      final PathStateMixtureDensityModel pathStateDist =
          pathStateEstimatorPredictor
              .createPredictiveDistribution(priorPredictiveMotionState);

      /*
       * We only consider non-zero predictive results, naturally.
       * Also, a path can be rejected due to direction,
       * so we must expect this.
       */
      if (pathStateDist.getDistributionCount() > 0) {
        final double mixtureLikelihood =
            pathStateDist.getPriorWeightSum();
        if (priorPredictiveMotionState.getInputDimensionality() == 4) {
          numberOfOffRoadPaths++;
        } else if (mixtureLikelihood > Double.NEGATIVE_INFINITY) {
          numberOfNonZeroPaths++;
        }

        weightsSum = LogMath.add(weightsSum, mixtureLikelihood);
        distributions.addAll(pathStateDist.getDistributions());
        weights =
            Doubles.concat(weights, pathStateDist.getPriorWeights());
      }
    }

    /*
     * Normalize over on/off-road and edge count
     */
    if (numberOfNonZeroPaths > 0 || numberOfOffRoadPaths > 0) {
      for (int i = 0; i < weights.length; i++) {
        /*
         * Normalized within categories (off and on-road)
         */
        if (distributions.get(i).getPathState().isOnRoad()) {
          weights[i] -= Math.log(numberOfNonZeroPaths);
        } else {
          weights[i] -= Math.log(numberOfOffRoadPaths);
        }
        /*
         * Normalize over categories
         */
        if (numberOfOffRoadPaths > 0) {
          weights[i] -= Math.log(2);
        }
      }
    }

    final PathStateMixtureDensityModel predictedPathStateDist =
        new PathStateMixtureDensityModel(distributions, weights);
    predictedState.setPathStateParam(SimpleBayesianParameter.create(
        state.getPathStateParam().getParameterPrior().getPathState(),
        predictedPathStateDist, state.getPathStateParam()
            .getParameterPrior()));

    return predictedState;
  }

}
