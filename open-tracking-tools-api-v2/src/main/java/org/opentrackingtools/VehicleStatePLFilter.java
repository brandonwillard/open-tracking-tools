package org.opentrackingtools;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.InferredPath;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathStateBelief;
import org.opentrackingtools.paths.impl.EdgePredictiveResults;
import org.opentrackingtools.paths.impl.InferredPathPrediction;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.model.WrappedWeightedValue;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleStatePLFilter extends AbstractParticleFilter<GpsObservation, VehicleState> {

  private long seed;
  private static final long serialVersionUID = -8257075186193062150L;

  private GpsObservation obs;
  private InferenceGraph inferredGraph;
  private VehicleStateInitialParameters parameters;
  private Boolean isDebug;
  private Random rng;

  public VehicleStatePLFilter(GpsObservation obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, ParticleFilter.Updater<GpsObservation, VehicleState> updater,
    Boolean isDebug, Random rng) {

    this.obs = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.isDebug = isDebug;
    this.rng = rng;
    this.setUpdater(updater);
  }

  @Override
  public void update(DataDistribution<VehicleState> target, GpsObservation obs) {
    final Multimap<VehicleState, WrappedWeightedValue<InferredPathPrediction>> stateToPaths = HashMultimap.create();
    final Set<InferredPath> evaluatedPaths = Sets.newHashSet();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final DefaultCountedDataDistribution<VehicleState> resampleDist =
        new DefaultCountedDataDistribution<VehicleState>(true);

    for (final VehicleState state : target.getDomain()) {

      final int count = ((DefaultCountedDataDistribution<VehicleState>) target).getCount(state);

      final Collection<InferredPath> instStateTransitions = inferredGraph.getPaths(state, obs);
      
      double timeDiff = (obs.getTimestamp().getTime() - state.getObservation().getTimestamp().getTime())/1000;

      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      double totalLogLik = Double.NEGATIVE_INFINITY;

      /*
       * Create one table to hold all pathEdges to their
       * likelihoods.  That way we can check for dups from
       * overlapping paths.
       * TODO determine if sharing this map between states is useful.
       */
      final Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik = Maps.newHashMap();

      final List<WrappedWeightedValue<InferredPathPrediction>> predictiveResults = Lists.newArrayList();
      for (final InferredPath path : instStateTransitions) {

        if (isDebug)
          evaluatedPaths.add(path);

        final InferredPathPrediction infPath =
            path.getPriorPredictionResults(this.inferredGraph, obs, state, edgeToPreBeliefAndLogLik);

        if (infPath != null) {
          totalLogLik = LogMath.add(totalLogLik, infPath.getTotalLogLikelihood());

          assert !Double.isNaN(totalLogLik);

          predictiveResults.add(new WrappedWeightedValue<InferredPathPrediction>(infPath, infPath
              .getTotalLogLikelihood()));
        }
      }

      stateToPaths.putAll(state, predictiveResults);

      resampleDist.increment(state, totalLogLik, count);
    }

    final Random rng = getRandom();

    Preconditions.checkState(!resampleDist.isEmpty());

    // TODO low-variance sampling?
    final ArrayList<? extends VehicleState> smoothedStates = resampleDist.sample(rng, getNumParticles());

    final DataDistribution<VehicleState> posteriorDist = new DefaultCountedDataDistribution<VehicleState>(true);

    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {

      /*
       * TODO: debug seeding 
       */
      this.seed = rng.nextLong();

      EdgePredictiveResults edgePredResults = sampleEdge(stateToPaths.get(state));

      final VehicleState newTransState = propagateStates(state, obs, edgePredResults, stateToPaths.size());

      ((DefaultCountedDataDistribution<VehicleState>) posteriorDist).increment(newTransState, 0d);

    }

    target.clear();
    ((DefaultCountedDataDistribution<VehicleState>) target).copyAll(posteriorDist);

    Preconditions
        .checkState(((DefaultCountedDataDistribution<VehicleState>) target).getTotalCount() == this.numParticles);

  }

  protected EdgePredictiveResults sampleEdge(Collection<WrappedWeightedValue<InferredPathPrediction>> weighedPaths) {

    final Random rng = getRandom();
    rng.setSeed(this.seed);

    final PathEdge posteriorEdge;
    final InferredPathPrediction sampledPathEntry;
    final EdgePredictiveResults predictionResults;
    /*
     * Sample a path
     */
    final DataDistribution<InferredPathPrediction> instStateDist =
        StatisticsUtil.getLogNormalizedDistribution(Lists.newArrayList(weighedPaths));

    sampledPathEntry = instStateDist.sample(rng);

    if (sampledPathEntry.getWeightedPathEdges().size() > 1) {
      /*
       * TODO FIXME: cache the creation of these distributions
       */
      final DataDistribution<PathEdge> pathEdgeDist =
          StatisticsUtil.getLogNormalizedDistribution(sampledPathEntry.getWeightedPathEdges());
      posteriorEdge = pathEdgeDist.sample(rng);

    } else {
      posteriorEdge = Iterables.getOnlyElement(sampledPathEntry.getWeightedPathEdges()).getValue();
    }
    predictionResults = Preconditions.checkNotNull(sampledPathEntry.getEdgeToPredictiveBelief().get(posteriorEdge));

    return predictionResults;
  }

  protected VehicleState propagateStates(VehicleState state, GpsObservation obs,
    EdgePredictiveResults predictionResults, int pathSupportSize) {

    final Random rng = getRandom();
    rng.setSeed(this.seed);

    final PathEdge posteriorEdge = predictionResults.getLocationPrediction().getEdge();
    final AbstractRoadTrackingFilter pathEstimator = state.getMovementFilter();
    /*
     * This is the belief that will be propagated.
     */
    final PathStateBelief priorPathStateBelief = predictionResults.getLocationPrediction().clone();
    final PathStateBelief updatedBelief =
        pathEstimator.measure(priorPathStateBelief, obs.getProjectedPoint(), posteriorEdge);

    /*
     * Update edge velocities
     * TODO should be offline...actually almost all of what follows should be.
     */
    updatedBelief.getPath().updateEdges(obs, updatedBelief.getGlobalStateBelief(), this.inferredGraph);

    /*
     * Update edge transition priors.
     */
    final OnOffEdgeTransDistribution updatedEdgeTransDist = state.getEdgeTransitionDist().clone();

    /*
     * Note: we don't want to update a transition like
     * off->off when there were no other choices.  This
     * would simply bias the results, and offer no real
     * benefit. 
     */
    if (pathSupportSize > 1) {
      updatedEdgeTransDist.update(state.getBelief().getEdge().getInferredEdge(), posteriorEdge.getInferredEdge());
    }

    /*
     * Update covariances, or not.
     */
    final AbstractRoadTrackingFilter updatedFilter = pathEstimator.clone();

    /*
     * To make sure that the samples connect when sampling
     * along paths in sequence, we need to adjust the prior
     */
    final PathStateBelief pathAdjustedPriorBelief = priorPathStateBelief;

    updatedFilter.update(state, obs, updatedBelief, pathAdjustedPriorBelief, rng);

    final VehicleState newTransState =
        this.inferredGraph.createVehicleState(obs, updatedFilter, updatedBelief.clone(), updatedEdgeTransDist.clone(),
            state);

    return newTransState;
  }
}