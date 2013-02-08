package org.opentrackingtools.statistics.filters.vehicles.particle_learning;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.EdgePredictiveResults;
import org.opentrackingtools.graph.paths.impl.InferredPathPrediction;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.statistics.distributions.impl.DefaultCountedDataDistribution;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.AbstractVehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.FilterInformation;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public abstract class AbstractVTPLFilter extends
    AbstractVehicleTrackingFilter {

  private long seed;
  private static final long serialVersionUID =
      -8257075186193062150L;

  public AbstractVTPLFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    AbstractVTParticleFilterUpdater updater, 
    Boolean isDebug, Random rng) {
    super(obs, inferredGraph, parameters, updater, isDebug, rng);
  }

  @Override
  protected void internalUpdate(
    DataDistribution<VehicleState> target, GpsObservation obs,
    double timeDiff) {
    final Multimap<VehicleState, WrappedWeightedValue<InferredPathPrediction>> stateToPaths =
        HashMultimap.create();
    final Set<InferredPath> evaluatedPaths =
        Sets.newHashSet();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final List<WrappedWeightedValue<VehicleState>> resampler =
        Lists.newArrayList();
    for (final VehicleState state : target.getDomain()) {

      final int count =
          ((DefaultCountedDataDistribution<VehicleState>) target)
              .getCount(state);

      final Collection<InferredPath> instStateTransitions =
          inferredGraph.getPaths(state, obs);

      state.getMovementFilter()
          .setCurrentTimeDiff(timeDiff);
      double totalLogLik = Double.NEGATIVE_INFINITY;

      /*
       * Create one table to hold all pathEdges to their
       * likelihoods.  That way we can check for dups from
       * overlapping paths.
       * TODO determine if sharing this map between states is useful.
       */
      final Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik =
          Maps.newHashMap();

      final List<WrappedWeightedValue<InferredPathPrediction>> predictiveResults = 
          Lists.newArrayList();
      for (final InferredPath path : instStateTransitions) {
        
        if (isDebug)
          evaluatedPaths.add(path);

//        if (state.getBelief().isOnRoad() && !path.isEmptyPath() 
//            && !Iterables.getFirst(path.getEdges(), null)
//            .getGeometry().equalsExact(
//                state.getBelief().getEdge().getGeometry())) {
//          continue;
//        }

        final InferredPathPrediction infPath =
            path.getPriorPredictionResults(obs, state,
                edgeToPreBeliefAndLogLik);
        
        if (infPath != null) {
          totalLogLik =
              LogMath.add(totalLogLik,
                  infPath.getTotalLogLikelihood());

          assert !Double.isNaN(totalLogLik);

          predictiveResults.add(
              new WrappedWeightedValue<InferredPathPrediction>(
                      infPath, infPath.getTotalLogLikelihood()));
        }
      }

      stateToPaths.putAll(state, predictiveResults);

      resampler.add(new WrappedWeightedValue<VehicleState>(
          state, totalLogLik, count));
    }

    final Random rng = getRandom();

    final DataDistribution<VehicleState> resampleDist =
        Preconditions.checkNotNull(
        StatisticsUtil
            .getLogNormalizedDistribution(resampler));

    // TODO low-variance sampling?
    final ArrayList<? extends VehicleState> smoothedStates =
        resampleDist.sample(rng, getNumParticles());

    if (isDebug)
      this.filterInfo.put(obs, new FilterInformation(
          evaluatedPaths, resampleDist, stateToPaths));

    final DataDistribution<VehicleState> posteriorDist =
        new DefaultCountedDataDistribution<VehicleState>();

    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {
        
      /*
       * TODO: debug seeding 
       */
      this.seed = rng.nextLong();

      final VehicleState newTransState =
          propagateStates(state, obs,
              stateToPaths.get(state));

      ((DefaultCountedDataDistribution<VehicleState>) posteriorDist)
          .increment(newTransState, 1d / numParticles);

    }

    target.clear();
    ((DefaultCountedDataDistribution<VehicleState>) target)
        .copyAll(posteriorDist);

    assert ((DefaultCountedDataDistribution<VehicleState>) target)
        .getTotalCount() == this.numParticles;

  }

  private
      VehicleState
      propagateStates(
        VehicleState state,
        GpsObservation obs,
        Collection<WrappedWeightedValue<InferredPathPrediction>> weighedPaths) {

    final Random rng = getRandom();
    rng.setSeed(this.seed);

    final PathEdge posteriorEdge;
    final InferredPathPrediction sampledPathEntry;
    final EdgePredictiveResults predictionResults;
    final AbstractRoadTrackingFilter sampledFilter;
    /*
     * Sample a path
     */
    final DataDistribution<InferredPathPrediction> instStateDist =
        StatisticsUtil.getLogNormalizedDistribution(Lists
            .newArrayList(weighedPaths));

    sampledPathEntry = instStateDist.sample(rng);

    if (sampledPathEntry.getPath().getPathEdges().size() > 1) {
      /*
       * TODO FIXME: cache the creation of these distributions
       */
      final DataDistribution<PathEdge> pathEdgeDist =
          StatisticsUtil
              .getLogNormalizedDistribution(sampledPathEntry
                  .getWeightedPathEdges());
      posteriorEdge = pathEdgeDist.sample(rng);

    } else {
      posteriorEdge =
          sampledPathEntry.getPath().getPathEdges().get(0);
    }
    predictionResults =
        sampledPathEntry.getEdgeToPredictiveBelief().get(
            posteriorEdge);
    sampledFilter = sampledPathEntry.getFilter();

    /*
     * Sample an edge directly
     */
    //    Map<PathEdge, InferredPathEntry> edgeToPathEntries = Maps.newHashMap();
    //    List<WrappedWeightedValue<PathEdge>> weighedEdges = Lists.newArrayList();
    //    for (WrappedWeightedValue<InferredPathEntry> weighedPath : weighedPaths) {
    //      weighedEdges.addAll(weighedPath.getValue().getWeightedPathEdges());
    //      for (WrappedWeightedValue<PathEdge> edge : weighedPath.getValue()
    //            .getWeightedPathEdges()) {
    //        edgeToPathEntries.put(edge.getValue(), weighedPath.getValue());
    //      }
    //    }
    //    final DataDistribution<PathEdge> instStateDist =
    //        StatisticsUtil.getLogNormalizedDistribution(weighedEdges);
    //    
    //    posteriorEdge = instStateDist.sample(rng);
    //    sampledPathEntry = edgeToPathEntries.get(posteriorEdge);
    //    sampledFilter = sampledPathEntry.getFilter();
    //    predictionResults = sampledPathEntry.getEdgeToPredictiveBelief().get(posteriorEdge);

    /*
     * This is the belief that will be propagated.
     */
    final PathStateBelief priorPathStateBelief =
        predictionResults.getLocationPrediction().clone();
    final PathStateBelief updatedBelief =
        sampledFilter.measure(priorPathStateBelief,
            obs.getProjectedPoint(), posteriorEdge);

    /*
     * Update edge velocities
     * TODO should be offline...actually almost all of what follows should be.
     */
    updatedBelief.getPath().updateEdges(obs,
        updatedBelief.getGlobalStateBelief(),
        this.inferredGraph);

    /*
     * Update edge transition priors.
     */
    final OnOffEdgeTransDirMulti updatedEdgeTransDist =
        state.getEdgeTransitionDist().clone();

    /*
     * Note: we don't want to update a transition like
     * off->off when there were no other choices.  This
     * would simply bias the results, and offer no real
     * benefit. 
     */
    if (instStateDist.size() > 1) {
      updatedEdgeTransDist.update(state.getBelief()
          .getEdge().getInferredEdge(),
          posteriorEdge.getInferredEdge());
    }

    /*
     * Update covariances, or not.
     */
    final AbstractRoadTrackingFilter updatedFilter =
        sampledFilter.clone();

    /*
     * To make sure that the samples connect when sampling
     * along paths in sequence, we need to adjust the prior
     */
    final PathStateBelief pathAdjustedPriorBelief =
        priorPathStateBelief;
//        updatedBelief.getPath().getStateBeliefOnPath(
//             priorPathStateBelief.getRawStateBelief());
//        SimplePathStateBelief.getPathStateBelief(updatedBelief
//            .getPath(),
//                 priorPathStateBelief.getRawStateBelief(),
//                 this.inferredGraph);

    updatedFilter.update(state, obs, updatedBelief,
        pathAdjustedPriorBelief, rng);

    final VehicleState newTransState =
        new VehicleState(this.inferredGraph, obs,
            updatedFilter, updatedBelief.clone(),
            updatedEdgeTransDist.clone(), state);

    return newTransState;
  }
}
