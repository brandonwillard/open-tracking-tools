package org.opentrackingtools;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;

import org.opentrackingtools.distributions.AdjMultivariateGaussian;
import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.distributions.ScaledInverseGammaCovDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.OnOffEdgeTransitionEstimatorPredictor;
import org.opentrackingtools.estimators.RoadMeasurementCovarianceEstimatorPredictor;
import org.opentrackingtools.estimators.RoadModelCovarianceEstimatorPredictor;
import org.opentrackingtools.estimators.TruncatedRoadKalmanFilter;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.updater.VehicleStatePLUpdater;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.model.MutableDoubleCount;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class VehicleStatePLFilter<O extends GpsObservation, G extends InferenceGraph>
    extends AbstractParticleFilter<O, VehicleStateDistribution<O>> {

  private static Logger _log = LoggerFactory
      .getLogger(VehicleStatePLFilter.class);
  private static final long serialVersionUID = -8257075186193062150L;

  protected final G inferredGraph;
  protected final Boolean isDebug;

  /*
   * Keep the last particle resample distribution (when debugging is on).
   * This distribution contains all the last evaluated transition states.
   */
  protected CountedDataDistribution<VehicleStateDistribution<O>> lastResampleDistribution;

  protected VehicleStateDistributionFactory<O, G> vehicleStateFactory;

  public VehicleStatePLFilter(O obs, G inferredGraph,
    VehicleStateDistributionFactory<O, G> vehicleStateFactory,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.vehicleStateFactory = vehicleStateFactory;
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleStatePLUpdater<O, G>(obs,
        inferredGraph, vehicleStateFactory, parameters, rng));
    this.setNumParticles(parameters.getNumParticles());
    this.setRandom(rng);
  }

  protected OnOffEdgeTransitionEstimatorPredictor
      getEdgeTransitionEstimatorPredictor(
        VehicleStateDistribution<O> updatedState,
        InferenceGraphEdge graphEdge) {
    return new OnOffEdgeTransitionEstimatorPredictor(updatedState,
        graphEdge);
  }

  public Boolean getIsDebug() {
    return this.isDebug;
  }

  public CountedDataDistribution<VehicleStateDistribution<O>>
      getLastResampleDistribution() {
    return this.lastResampleDistribution;
  }

  protected RoadMeasurementCovarianceEstimatorPredictor
      getRoadMeasurementCovarianceEstimatorPredictor(
        VehicleStateDistribution<O> updatedState,
        Vector newObsStateSample) {
    return new RoadMeasurementCovarianceEstimatorPredictor(
        updatedState, newObsStateSample);
  }

  protected RoadModelCovarianceEstimatorPredictor
      getRoadModelCovarianceEstimatorPredictor(
        VehicleStateDistribution<O> updatedState,
        MotionStateEstimatorPredictor motionStateEstimatorPredictor) {
    return new RoadModelCovarianceEstimatorPredictor(updatedState,
        motionStateEstimatorPredictor, this.random);
  }

  /**
   * This method takes a prior predictive vehicle state distribution and returns
   * a distribution over its possible transition states, with prior predictive
   * likelihood weights.
   * 
   * @param predictedState
   * @param obs
   * @return
   */
  protected CountedDataDistribution<VehicleStateDistribution<O>>
      internalPriorPrediction(
        VehicleStateDistribution<O> predictedState, O obs) {
    final CountedDataDistribution<VehicleStateDistribution<O>> childDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

    final PathStateMixtureDensityModel predictedPathStateMixture =
        predictedState.getPathStateParam()
            .getConditionalDistribution();

    for (int i = 0; i < predictedPathStateMixture
        .getDistributionCount(); i++) {
      final PathStateDistribution predictedPathStateDist =
          predictedPathStateMixture.getDistributions().get(i);
      final double pathStateDistLogLikelihood =
          predictedPathStateMixture.getPriorWeights()[i];
      final double edgeTransitionLogLikelihood =
          predictedState
              .getEdgeTransitionParam()
              .getConditionalDistribution()
              .getProbabilityFunction()
              .logEvaluate(predictedPathStateDist.getPathState().getEdge().getInferenceGraphSegment());

      /*
       * Create a new vehicle state for this possible transition
       * path state, and, since it starts as a copy of the original
       * state, make sure we update the pertinent parameters.
       */
      final VehicleStateDistribution<O> predictedChildState =
          predictedState.clone();

      predictedChildState.setPathStateParam(SimpleBayesianParameter
          .create(predictedPathStateDist.getPathState(),
              predictedPathStateMixture, predictedPathStateDist));

      final MultivariateGaussian measurementPredictionDist =
          predictedState.getMotionStateEstimatorPredictor()
              .getMeasurementDistribution(predictedPathStateDist);

      predictedChildState.setMotionStateParam(SimpleBayesianParameter
          .create(measurementPredictionDist.getMean(),
              measurementPredictionDist,
              predictedPathStateDist.getMotionDistribution()));

      final double obsLogLikelihood =
          this.getUpdater().computeLogLikelihood(predictedChildState,
              obs);

      predictedChildState.setParentState(predictedState
          .getParentState());

      /*
       * Set these for debugging.
       */
      predictedChildState
          .setObsLogLikelihood(obsLogLikelihood);
      predictedChildState
          .setPathStateDistLogLikelihood(pathStateDistLogLikelihood);
      predictedChildState
          .setEdgeTransitionLogLikelihood(edgeTransitionLogLikelihood);

      final double childLogLikTotal =
          obsLogLikelihood + pathStateDistLogLikelihood
              + edgeTransitionLogLikelihood;

      childDist.increment(predictedChildState, childLogLikTotal);
    }

    return childDist;
  }

  /**
   * This method performs the Bayes update for a single vehicle state.
   * 
   * @param state
   * @param obs
   * @return
   */
  protected VehicleStateDistribution<O> internalUpdate(
    VehicleStateDistribution<O> state, O obs) {
    final VehicleStateDistribution<O> updatedState = state.clone();
    final PathStateDistribution priorPredictivePathStateDist =
        updatedState.getPathStateParam().getParameterPrior().clone();

    /*
     * Update motion and path state by first updating the ground
     * coordinates, then reprojecting onto the edge.
     * Scratch that.
     * For on-road, update by projecting the observation onto the path, then
     * perform a 2d update.
     */
    final MultivariateGaussian updatedMotionState =
        priorPredictivePathStateDist.getMotionDistribution().clone();
    final PathStateDistribution posteriorPathStateDist;
    if (priorPredictivePathStateDist.getPathState().isOnRoad()) {
      final TruncatedRoadKalmanFilter roadFilter =
          updatedState.getMotionStateEstimatorPredictor()
              .getRoadFilter().clone();

      final AdjMultivariateGaussian obsProj =
          PathUtils
              .getRoadObservation(obs.getProjectedPoint(),
                  updatedState.getObservationCovarianceParam()
                      .getValue(), priorPredictivePathStateDist
                      .getPathState().getPath(),
                  priorPredictivePathStateDist.getPathState()
                      .getEdge());
      
      /*
       * Make sure the projected obs is after or at the current location
       * along the edge; otherwise, we'll end up allowing backward movement.
       */
      final double currentDist = updatedMotionState.getMean().getElement(0);
      if (obsProj.getMean().getElement(0) < currentDist) {
        obsProj.getMean().setElement(0, currentDist);
      }

      roadFilter.setMeasurementCovariance(obsProj.getCovariance());
      roadFilter.measure(updatedMotionState, obsProj.getMean());
      Preconditions.checkState(updatedMotionState.getCovariance().getElement(0, 0) > 1e-6);
      /*
       * Stay on our edge/path.  Note: we must, since we're conditioning on it.
       */
      final Path postPath = priorPredictivePathStateDist
              .getPathState().getPath();
      updatedMotionState.setMean( 
         Iterables.getLast(postPath.getPathEdges()).clampToEdge(updatedMotionState.getMean()));
      posteriorPathStateDist = new PathStateDistribution(postPath, updatedMotionState);
    } else {
      updatedState.getMotionStateEstimatorPredictor().update(
          updatedMotionState, obs.getProjectedPoint());
      posteriorPathStateDist =
          new PathStateDistribution(priorPredictivePathStateDist
              .getPathState().getPath(), updatedMotionState);
    }

    /*
     * Update parameters
     */
    updatedState.getPathStateParam().setValue(
        posteriorPathStateDist.getPathState());
    updatedState.getPathStateParam().setParameterPrior(
        posteriorPathStateDist);

    Vector newObsStateSample;

    final RoadModelCovarianceEstimatorPredictor modelCovarianceEstimator =
        new RoadModelCovarianceEstimatorPredictor(updatedState,
            state.getMotionStateEstimatorPredictor(), this.random);
    final ScaledInverseGammaCovDistribution currentModelCovDistribution =
        (ScaledInverseGammaCovDistribution) (posteriorPathStateDist.getPathState().isOnRoad()
        ? updatedState.getOnRoadModelCovarianceParam()
            .getParameterPrior().clone() : updatedState
            .getOffRoadModelCovarianceParam()
            .getParameterPrior().clone());
    modelCovarianceEstimator.update(currentModelCovDistribution,
        obs.getProjectedPoint());

    /*
     * After updating the covariance priors, we need to sample our new covariance matrix.
     * Also, note that we really have separate on/off covariances.  We could project
     * back and forth, then we'd really only have one.
     */
    if (posteriorPathStateDist.getPathState().isOnRoad()) {
      updatedState.getOnRoadModelCovarianceParam()
          .setParameterPrior(currentModelCovDistribution);
      final Matrix stateCovSample =
          currentModelCovDistribution.sample(this.random);
      updatedState.getOnRoadModelCovarianceParam().setValue(
          stateCovSample);
      updatedState.getOnRoadModelCovarianceParam()
          .setConditionalDistribution(
              new MultivariateGaussian(VectorFactory.getDefault()
                  .createVector1D(), stateCovSample));
    } else {
      updatedState.getOffRoadModelCovarianceParam()
          .setParameterPrior(currentModelCovDistribution);
      final Matrix stateCovSample =
          currentModelCovDistribution.sample(this.random);
      updatedState.getOffRoadModelCovarianceParam().setValue(
          stateCovSample);
      updatedState.getOffRoadModelCovarianceParam()
          .setConditionalDistribution(
              new MultivariateGaussian(VectorFactory.getDefault()
                  .createVector1D(), stateCovSample));
    }
    newObsStateSample =
        MotionStateEstimatorPredictor.getOg().times(
            modelCovarianceEstimator.getNewPosteriorStateSample()
                .getGroundDistribution().getMean());

    final RoadMeasurementCovarianceEstimatorPredictor measurementCovarianceEstimator =
        this.getRoadMeasurementCovarianceEstimatorPredictor(
            updatedState, newObsStateSample);
    final ScaledInverseGammaCovDistribution currentObsCovDistribution =
        (ScaledInverseGammaCovDistribution) updatedState.getObservationCovarianceParam()
        .getParameterPrior().clone();
    measurementCovarianceEstimator.update(
        currentObsCovDistribution, obs.getProjectedPoint());

    updatedState.getObservationCovarianceParam().setParameterPrior(
        currentObsCovDistribution);
    final Matrix obsCovSample =
        currentObsCovDistribution.sample(this.random);
    updatedState.getObservationCovarianceParam().setValue(
        obsCovSample);
    updatedState.getObservationCovarianceParam()
        .setConditionalDistribution(
            new MultivariateGaussian(VectorFactory.getDefault()
                .createVector(obsCovSample.getNumColumns()),
                obsCovSample));

    updatedState.getMotionStateParam().setParameterPrior(
        posteriorPathStateDist.getMotionDistribution());
    final MultivariateGaussian obsMotionDist =
        updatedState.getMotionStateEstimatorPredictor()
            .getObservationDistribution(
                posteriorPathStateDist.getMotionDistribution(),
                posteriorPathStateDist.getPathState().getEdge());
    updatedState.getMotionStateParam().setConditionalDistribution(
        obsMotionDist);
    updatedState.getMotionStateParam().setValue(
        obsMotionDist.getMean());

    final OnOffEdgeTransPriorDistribution updatedEdgeTransPrior =
        updatedState.getEdgeTransitionParam().getParameterPrior()
            .clone();
    /*
     * Note that this is only updating the transition from start
     * edge to final edge on the path.
     */
    final InferenceGraphEdge fromEdge =
        updatedState.getParentState().getPathStateParam().getValue()
            .getEdge().getInferenceGraphSegment();
    final InferenceGraphEdge toEdge =
        updatedState.getPathStateParam().getValue()
            .getEdge().getInferenceGraphSegment();
    final OnOffEdgeTransitionEstimatorPredictor edgeTransitionEstimatorPredictor =
        this.getEdgeTransitionEstimatorPredictor(updatedState,
            fromEdge);
    edgeTransitionEstimatorPredictor.update(updatedEdgeTransPrior,
        toEdge);

    final OnOffEdgeTransDistribution updatedEdgeTransConditional =
        new OnOffEdgeTransDistribution(this.inferredGraph, updatedState.getPathStateParam().getValue(), 
            toEdge, updatedState.getObservationCovarianceParam().getValue(), 
            updatedEdgeTransPrior.getEdgeMotionTransProbPrior().getMean(),
            updatedEdgeTransPrior.getFreeMotionTransProbPrior().getMean());
    
    updatedState.setEdgeTransitionParam(SimpleBayesianParameter
        .create(updatedEdgeTransPrior.getMean(),
            updatedEdgeTransConditional, updatedEdgeTransPrior));

    updatedState.setEdgeTransitionParam(SimpleBayesianParameter
        .create(updatedEdgeTransPrior.getMean(),
            updatedEdgeTransConditional, updatedEdgeTransPrior));

    return updatedState;
  }

  private
      void
      printResampleDist(
        CountedDataDistribution<VehicleStateDistribution<O>> resampleDist,
        VehicleStateDistribution<O> trueState, boolean useResampleDist) {
    final List<Entry<VehicleStateDistribution<O>, MutableDouble>> sortedDist =
        Lists.newArrayList(resampleDist.asMap().entrySet());
    Collections
        .sort(
            sortedDist,
            new Comparator<Entry<VehicleStateDistribution<O>, MutableDouble>>() {
              @Override
              public int compare(
                Entry<VehicleStateDistribution<O>, MutableDouble> o1,
                Entry<VehicleStateDistribution<O>, MutableDouble> o2) {
                final double adjValue1 = o1.getValue().getValue();
                final double adjValue2 = o2.getValue().getValue();
                return -Double.compare(adjValue1, adjValue2);
              }
            });
    for (final Entry<VehicleStateDistribution<O>, MutableDouble> entry : sortedDist) {
      final VehicleStateDistribution<O> state =
          useResampleDist ? entry.getKey()
              .getTransitionStateDistribution().getMaxValueKey()
              : entry.getKey();
      final Vector distToTrueState =
          trueState != null ? state
              .getPathStateParam()
              .getParameterPrior()
              .getGroundDistribution()
              .getMean()
              .minus(
                  trueState.getPathStateParam().getValue()
                      .getGroundState()) : null;
      final int count =
          ((MutableDoubleCount) entry.getValue()).getCount();
      final double adjValue = entry.getValue().getValue();
      System.out.println(resampleDist.getLogFraction(entry.getKey())
          + " [" + adjValue + "] (" + count + "),\n\t ["
          + distToTrueState + "]:\n\t"
          + state.getPathStateParam().getValue()
          + "\n\t pathStateLik="
          + state.getPathStateDistLogLikelihood() + "\n\t obsLik="
          + state.getObsLogLikelihood());
    }
  }

  public
      void
      setLastResampleDistribution(
        CountedDataDistribution<VehicleStateDistribution<O>> lastResampleDistribution) {
    this.lastResampleDistribution = lastResampleDistribution;
  }

  @Override
  public void update(
    DataDistribution<VehicleStateDistribution<O>> target, O obs) {

    /*
     * Compute predictive distributions, and create a distribution out of those and
     * their likelihoods for the new observation.
     */
    final CountedDataDistribution<VehicleStateDistribution<O>> resampleDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

    for (final VehicleStateDistribution<O> state : target.getDomain()) {

      final int count;
      if (target instanceof CountedDataDistribution<?>) {
        count = ((CountedDataDistribution) target).getCount(state);
      } else {
        count = 1;
      }
      Preconditions.checkState(count > 0);

      final double logCount = Math.log(count);

      VehicleStateDistribution<O> predictedState = state.clone();
      predictedState.setParentState(state);
      predictedState.setObservation(obs);
      predictedState = this.updater.update(predictedState);

      final CountedDataDistribution<VehicleStateDistribution<O>> childDist =
          this.internalPriorPrediction(predictedState, obs);
      predictedState.setTransitionStateDistribution(childDist);

      resampleDist.increment(predictedState, childDist.getTotal()
          + logCount);

    }

    Preconditions.checkState(!resampleDist.isEmpty());

    if (this.isDebug) {
      this.lastResampleDistribution = resampleDist;
    }

    /*
     * Resample the predictive distributions.  Now we're dealing with the "best" states.
     */
    final ArrayList<VehicleStateDistribution<O>> smoothedStates =
        resampleDist.sample(this.random, this.getNumParticles());

    final List<VehicleStateDistribution<O>> updatedStates =
        Lists.newArrayList();

    /*
     * Propagate/smooth the best states. 
     */
    for (final VehicleStateDistribution<O> state : smoothedStates) {
      final VehicleStateDistribution<O> sampledTransitionState =
          state.getTransitionStateDistribution().sample(this.random);
      final VehicleStateDistribution<O> updatedState =
          this.internalUpdate(sampledTransitionState, obs);
      if (this.isDebug) {
        updatedState.setTransitionStateDistribution(state
            .getTransitionStateDistribution());
        updatedState.setPriorPredictiveState(sampledTransitionState);
      }
      updatedStates.add(updatedState);
    }

    target.clear();
    target.incrementAll(updatedStates);

    Preconditions.checkState(target.getDomainSize() > 0);
    if (target instanceof CountedDataDistribution<?>) {
      Preconditions.checkState(((CountedDataDistribution<?>) target)
          .getTotalCount() == this.numParticles);
    }

  }

}