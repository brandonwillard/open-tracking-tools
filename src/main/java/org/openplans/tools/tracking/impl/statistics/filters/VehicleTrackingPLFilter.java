package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.WrappedWeightedValue;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath.EdgePredictiveResults;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.DataCube;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.base.Predicate;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleTrackingPLFilter extends
    AbstractVehicleTrackingFilter {

  private static final long serialVersionUID = -8257075186193062150L;

  public VehicleTrackingPLFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPLFilterUpdater(obs,
            inferredGraph, parameters), isDebug);
  }

  @Override
  protected void internalUpdate(
    DataDistribution<VehicleState> target, Observation obs,
    double timeDiff) {
    final Multimap<VehicleState, WrappedWeightedValue<InferredPathEntry>> stateToPaths =
        HashMultimap.create();
    final Set<InferredPath> evaluatedPaths = Sets.newHashSet();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final List<WrappedWeightedValue<VehicleState>> resampler =
        Lists.newArrayList();
    for (final VehicleState state : target.getDomain()) {

      final int count =
          ((DefaultCountedDataDistribution<VehicleState>) target)
              .getCount(state);

      final Set<InferredPath> instStateTransitions =
          inferredGraph.getPaths(state, obs.getObsPoint());

      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      double totalLogLik = Double.NEGATIVE_INFINITY;

      /*
       * Create one table to hold all pathEdges to their
       * likelihoods.  That way we can check for dups from
       * overlapping paths.
       * TODO determine if sharing this map between states is useful.
       */
      final Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik =
          Maps.newHashMap();

      for (final InferredPath path : instStateTransitions) {
      
        if (isDebug)
          evaluatedPaths.add(path);

        final InferredPathEntry infPath =
            path.getPredictiveLogLikelihood(obs, state,
                edgeToPreBeliefAndLogLik);

        if (infPath != null) {
          totalLogLik =
              LogMath.add(totalLogLik,
                  infPath.getTotalLogLikelihood());

          assert !Double.isNaN(totalLogLik);

          stateToPaths.put(state,
              new WrappedWeightedValue<InferredPathEntry>(infPath,
                  infPath.getTotalLogLikelihood()));
        }
      }

      resampler.add(new WrappedWeightedValue<VehicleState>(state,
          totalLogLik, count));
    }

    final Random rng = getRandom();

    final DataDistribution<VehicleState> resampleDist =
        StatisticsUtil.getLogNormalizedDistribution(resampler);

    // TODO low-variance sampling?
    final ArrayList<? extends VehicleState> smoothedStates =
        resampleDist.sample(rng, getNumParticles());

    if (isDebug)
      this.filterInfo.put(obs, new FilterInformation(evaluatedPaths,
          resampleDist));

    final DataDistribution<VehicleState> posteriorDist =
        new DefaultCountedDataDistribution<VehicleState>();
    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {

      final DataDistribution<InferredPathEntry> instStateDist =
          StatisticsUtil.getLogNormalizedDistribution(Lists
              .newArrayList(stateToPaths.get(state)));
      final InferredPathEntry sampledPathEntry =
          instStateDist.sample(rng);

      /*
       * This is a bit confusing, so really try to understand this:
       * The edge we're about to sample is not necessarily the edge that our filtering
       * says we should be on.  The edges, in this case, only correspond to stretches of
       * length-locations that were evaluated.  The posterior/filtering result that we
       * obtain from these edges will adjust our previous length-location relative to how good
       * it would've/could've been to be on each edge.  Essentially, this is kind of like saying
       * that we have to walk to that better edge relative to how fast we are, not simply teleport.
       */
      final PathEdge posteriorEdge;
      if (sampledPathEntry.getPath().getEdges().size() > 1) {
        final DataDistribution<PathEdge> pathEdgeDist =
            StatisticsUtil
                .getLogNormalizedDistribution(sampledPathEntry
                    .getWeightedPathEdges());
        posteriorEdge = pathEdgeDist.sample(rng);
        
      } else {
        posteriorEdge = sampledPathEntry
                .getPath().getEdges().get(0);
      }
      
      final InferredPath posteriorPath;
      if (!posteriorEdge.isEmptyEdge()) {
        /*
         * IMPORTANT:
         * We need to construct the subpath that will actually be used, otherwise
         * we can end up on parts of the path not corresponding to the likelihood
         * that got us this sample.
         */
        posteriorPath = InferredPath.getInferredPath(
            Lists.newArrayList(
              Iterables.filter(sampledPathEntry.getPath().getEdges(), new Predicate<PathEdge>() {
                @Override
                public boolean apply(@Nullable PathEdge input) {
                  return Double.compare(Math.abs(input.getDistToStartOfEdge()), 
                      Math.abs(posteriorEdge.getDistToStartOfEdge())) <= 0;
                }
                
              })), 
            sampledPathEntry.getPath().getIsBackward());
      } else {
        posteriorPath = InferredPath.getEmptyPath();
      }
      

      /*
       * This belief is p(x_{t+1} | Z_t, ..., y_{t+1}).  The dependency
       * on y_{t+1} obtained through resampling based on the likelihoods.
       */
      final MultivariateGaussian sampledBelief =
          sampledPathEntry.getEdgeToPredictiveBelief()
              .get(posteriorEdge)
              .getWeightedPredictiveDist().getValue();

      /*
       * This is the belief that will be propagated.
       */
      final PathStateBelief priorPathStateBelief = PathStateBelief.getPathStateBelief(
          posteriorPath, sampledBelief.clone());
      final PathStateBelief updatedBelief  = sampledPathEntry
          .getFilter().measure(priorPathStateBelief, 
              obs.getProjectedPoint(), posteriorEdge);

      
      /*
       * Update edge velocities
       * TODO should be offline...actually almost all of what follows should be.
       */
      posteriorPath.updateEdges(obs, updatedBelief.getStateBelief(), this.inferredGraph);
      
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
        updatedEdgeTransDist.update(state.getInferredEdge(),
              posteriorEdge.getInferredEdge());
      }
  
      /*
       * Update covariances, or not.
       */
      final AbstractRoadTrackingFilter<?> updatedFilter =
          sampledPathEntry.getFilter().clone();

      updatedFilter.updateSufficientStatistics(obs, 
          updatedBelief, priorPathStateBelief, rng);

      final VehicleState newTransState =
          new VehicleState(this.inferredGraph, obs, updatedFilter,
              updatedBelief.getStateBelief().clone(), updatedEdgeTransDist.clone(),
              posteriorPath, state);

      ((DefaultCountedDataDistribution<VehicleState>) posteriorDist)
          .increment(newTransState, 1d / numParticles);
  
    }

    target.clear();
    ((DefaultCountedDataDistribution<VehicleState>) target)
        .copyAll(posteriorDist);

    assert ((DefaultCountedDataDistribution<VehicleState>) target)
        .getTotalCount() == this.numParticles;

  }
}
