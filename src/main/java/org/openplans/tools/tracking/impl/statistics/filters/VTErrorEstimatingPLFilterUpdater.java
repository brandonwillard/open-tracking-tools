package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;

import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.commons.lang.NotImplementedException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Sets;

public class VTErrorEstimatingPLFilterUpdater implements
    PLParticleFilterUpdater<Observation, VehicleState> {

  private static final long serialVersionUID = 2884138088944317656L;

  private final Observation initialObservation;

  private final OtpGraph inferredGraph;

  private final VehicleStateInitialParameters parameters;

  private Random random;

  public VTErrorEstimatingPLFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
  }

  @Override
  public VTErrorEstimatingPLFilterUpdater clone() {
    throw new NotImplementedException();
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    throw new NotImplementedException();
    //    return particle.getProbabilityFunction().logEvaluate(
    //        new VehicleStateConditionalParams(PathEdge.getEdge(
    //            particle.getInferredEdge(), 0d), observation
    //            .getProjectedPoint(), 0d));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {

    final ErrorEstimatingRoadTrackingFilter tmpTrackingFilter =
        new ErrorEstimatingRoadTrackingFilter(
            parameters.getObsCov(), parameters.getObsCovDof(),
            parameters.getOffRoadStateCov(), parameters.getOffRoadCovDof(),
            parameters.getOnRoadStateCov(), parameters.getOnRoadCovDof(), 
            parameters.getInitialObsFreq() , this.random);

    final MultivariateGaussian tmpInitialBelief =
        tmpTrackingFilter.createInitialLearnedObject();
    final Vector xyPoint = initialObservation.getProjectedPoint();
    tmpInitialBelief.setMean(VectorFactory.getDefault().copyArray(
        new double[] { xyPoint.getElement(0), 0d,
            xyPoint.getElement(1), 0d }));
    final List<StreetEdge> initialEdges =
        inferredGraph.getNearbyEdges(tmpInitialBelief,
            tmpTrackingFilter);

    final DataDistribution<VehicleState> initialDist =
        new DefaultDataDistribution<VehicleState>(numParticles);

    final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();
    if (!initialEdges.isEmpty()) {
      for (final Edge nativeEdge : initialEdges) {
        final InferredEdge edge =
            inferredGraph.getInferredEdge(nativeEdge);
        final PathEdge pathEdge = PathEdge.getEdge(edge, 0d, false);
        final InferredPath path =
            InferredPath.getInferredPath(pathEdge);
        evaluatedPaths.add(new InferredPathEntry(path, null, null,
            null, Double.NEGATIVE_INFINITY));

        final ErrorEstimatingRoadTrackingFilter trackingFilter =
            new ErrorEstimatingRoadTrackingFilter(
              parameters.getObsCov(), parameters.getObsCovDof(),
              parameters.getOffRoadStateCov(), parameters.getOffRoadCovDof(),
              parameters.getOnRoadStateCov(), parameters.getOnRoadCovDof(), 
              parameters.getInitialObsFreq() , this.random);

        final OnOffEdgeTransDirMulti edgeTransDist =
            new OnOffEdgeTransDirMulti(inferredGraph,
                parameters.getOnTransitionProbs(),
                parameters.getOffTransitionProbs());

        final VehicleState state =
            new VehicleState(this.inferredGraph, initialObservation,
                pathEdge.getInferredEdge(), trackingFilter,
                edgeTransDist, this.random);

        /*
         * Make sure we have a state sample for updating
         */
        final Vector stateSample =
            trackingFilter.sampleStateTransition(state.getBelief()
                .getMean(), state.getPath(), this.random);
        trackingFilter
            .setCurrentStateSample(new DefaultPair<Vector, InferredEdge>(
                stateSample, state.getEdge()));

        /*
         * Sample an initial prior for the transition probabilities
         */
        final Vector edgePriorParams =
            state.getEdgeTransitionDist()
                .getEdgeMotionTransProbPrior().sample(this.random);
        final Vector freePriorParams =
            state.getEdgeTransitionDist()
                .getFreeMotionTransProbPrior().sample(this.random);
        state.getEdgeTransitionDist().getEdgeMotionTransPrior()
            .setParameters(edgePriorParams);
        state.getEdgeTransitionDist().getFreeMotionTransPrior()
            .setParameters(freePriorParams);

        final double lik =
            state.getProbabilityFunction().evaluate(
                initialObservation);

        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final ErrorEstimatingRoadTrackingFilter trackingFilter =
        new ErrorEstimatingRoadTrackingFilter(
              parameters.getObsCov(), parameters.getObsCovDof(),
              parameters.getOffRoadStateCov(), parameters.getOffRoadCovDof(),
              parameters.getOnRoadStateCov(), parameters.getOnRoadCovDof(), 
              parameters.getInitialObsFreq() , this.random);

    final OnOffEdgeTransDirMulti edgeTransDist =
        new OnOffEdgeTransDirMulti(inferredGraph,
            parameters.getOnTransitionProbs(),
            parameters.getOffTransitionProbs());

    final VehicleState state =
        new VehicleState(this.inferredGraph, initialObservation,
            InferredEdge.getEmptyEdge(), trackingFilter,
            edgeTransDist, this.random);

    /*
     * Make sure we have a state sample for updating
     */
    final Vector stateSample =
        trackingFilter.sampleStateTransition(state.getBelief()
            .getMean(), state.getPath(), this.random);
    trackingFilter
        .setCurrentStateSample(new DefaultPair<Vector, InferredEdge>(
            stateSample, state.getEdge()));

    /*
     * Sample an initial prior for the transition probabilities
     */
    final Vector edgeDriorParams =
        state.getEdgeTransitionDist().getEdgeMotionTransProbPrior()
            .sample(this.random);
    final Vector freeDriorParams =
        state.getEdgeTransitionDist().getFreeMotionTransProbPrior()
            .sample(this.random);
    state.getEdgeTransitionDist().getEdgeMotionTransPrior()
        .setParameters(edgeDriorParams);
    state.getEdgeTransitionDist().getFreeMotionTransPrior()
        .setParameters(freeDriorParams);

    final double lik =
        state.getProbabilityFunction().evaluate(initialObservation);

    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>(
            initialDist.sample(this.random, numParticles));

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Override
  public Random getRandom() {
    return random;
  }

  @Override
  public void setRandom(Random rng) {
    this.random = rng;
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

}