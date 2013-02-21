package org.opentrackingtools.statistics.filters.vehicles.impl;

import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nullable;

import org.apache.commons.lang.NotImplementedException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ForwardMovingRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Envelope;

public class VehicleTrackingPathSamplerFilterUpdater extends
    AbstractVTParticleFilterUpdater {

  private static final long serialVersionUID =
      2884138088944317656L;
  private static final long maxGraphBoundsResampleTries = (long) 1e6;

  private static final Logger _log = LoggerFactory
      .getLogger(VehicleTrackingPathSamplerFilterUpdater.class);
  
  public VehicleTrackingPathSamplerFilterUpdater(
      GpsObservation obs,
      InferenceGraph inferredGraph,
      VehicleStateInitialParameters parameters,
      Random rng) throws ClassNotFoundException, SecurityException, NoSuchMethodException, IllegalArgumentException, InstantiationException, IllegalAccessException, InvocationTargetException {
    super(obs, inferredGraph, parameters, rng);
    
  }

  @Override
  public double computeLogLikelihood(VehicleState particle,
    GpsObservation observationFactory) {
    return particle.getProbabilityFunction().logEvaluate(
        observationFactory);
  }

  @Override
  public DataDistribution<VehicleState>
      createInitialParticles(int numParticles) {

    final DataDistribution<VehicleState> initialDist =
        super.createInitialParticles(numParticles);
    for (final VehicleState state : initialDist.getDomain()) {
      /**
       * I hate doing this, but since these are state instances, they don't have
       * distributions, so no variances. TODO FIXME: we should have a better
       * design. one that take the above into account.
       */
      state
          .getBelief()
          .getLocalStateBelief()
          .setCovarianceInverse(
              MatrixFactory.getDiagonalDefault()
                  .createMatrix(
                      state.getBelief().getLocalState()
                          .getDimensionality(),
                      state.getBelief().getLocalState()
                          .getDimensionality()));
      state
          .getBelief()
          .getGlobalStateBelief()
          .setCovarianceInverse(
              MatrixFactory.getDiagonalDefault()
                  .createMatrix(
                      state.getBelief().getLocalState()
                          .getDimensionality(),
                      state.getBelief().getLocalState()
                          .getDimensionality()));
    }

    return initialDist;
  }

  private Collection<? extends InferredEdge>
      getTransferableEdges(InferredEdge inferredEdge,
        Vector newState) {
    /*
     * We can evaluate transfers from the exact
     * source edge, or from all topologically equivalent
     * edges from the source edge.
     */
    final Set<InferredEdge> transferEdges =
        Sets.newHashSet();

    if (newState.getElement(0) < 0d) {
//      transferEdges.addAll(
//          this.inferenceGraph.getIncomingTransferableEdges(inferredEdge));
    } else if (newState.getElement(0) > 0d) {
      transferEdges.addAll(this.inferenceGraph
          .getOutgoingTransferableEdges(inferredEdge));
    } else {
//      transferEdges.addAll(this.inferenceGraph
//          .getIncomingTransferableEdges(inferredEdge));
//      transferEdges.addAll(this.inferenceGraph
//          .getOutgoingTransferableEdges(inferredEdge));
    }
    return transferEdges;
  }

  /**
   * Sample a path for the given parameters.<br>
   * The method is simple:<br>
   * <ul>
   * <li>For road-states, sample a transition error in order to tell the
   * direction of movement. Then we determine the possible transition edges (for
   * ground-states these are the snapped roads).</li>
   * <li>Sample a transition edge. This could change our state-type (ground,
   * road). Then project our state, and finish if we're in a ground-state.</li>
   * <li>If we're here then we're in a road-state and we continue sampling edges
   * (excluding off-road transitions) until we've traveled the projected
   * distance.</li>
   * </ul>
   * 
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  public PathStateBelief sampleNextState(VehicleState currentState) {

    final Random rng = this.random;
    rng.setSeed(this.seed);

    PathStateBelief currentStateBelief = currentState.getBelief();
    OnOffEdgeTransDirMulti edgeTransDist = currentState.getEdgeTransitionDist();
    AbstractRoadTrackingFilter movementFilter = currentState.getMovementFilter();
    
    PathEdge newEdge =
        this.inferenceGraph.getPathEdge(currentStateBelief.getEdge()
            .getInferredEdge(), 0d, currentStateBelief.getPath()
            .isBackward());
    PathEdge lastNewEdge = null;
    MultivariateGaussian newStateDist = currentStateBelief.getLocalStateBelief().clone();

    final List<PathEdge> newPathEdges = Lists.newArrayList();

    /*
     * This will signify the start distance on the
     * initial edge in the new direction of motion.
     */
    double initialLocation = 0d;
    double distTraveled = 0d;
    Double totalDistToTravel = null;
    while (totalDistToTravel == null
        || Math.abs(totalDistToTravel) > Math
            .abs(distTraveled)) {

      final List<InferredEdge> transferEdges =
          Lists.newArrayList();
      if (newEdge.isNullEdge()) {
        final Vector projLocation =
            AbstractRoadTrackingFilter.getOg().times(
                newStateDist.getMean());
        final double radius =
            StatisticsUtil
                .getLargeNormalCovRadius((DenseMatrix) movementFilter
                    .getObsCovar());
        for (final InferredEdge edge : this.inferenceGraph
            .getNearbyEdges(projLocation, radius)) {
          transferEdges.add(edge);
        }
      } else {
        if (totalDistToTravel == null) {

          /*
           * XXX: Here we're allowing ourself to consider having
           * been on another topologically equivalent edge as our
           * source edge.  This way the graph traversal restrictions
           * are mostly removed. 
           * This should probably be removed when using the bootstrap
           * filter in production.
           */
//          final Set<InferredEdge> equivEdges =
//              this.inferenceGraph
//                  .getTopoEquivEdges(initialState.getEdge()
//                      .getInferredEdge());
//          transferEdges.addAll(equivEdges);
          transferEdges.add(currentStateBelief.getEdge().getInferredEdge());

          /*
           * We only allow going off-road when starting a path.
           */
          transferEdges.add(this.inferenceGraph.getNullInferredEdge());
        } else {
          transferEdges.addAll(getTransferableEdges(
              newEdge.getInferredEdge(), newStateDist.getMean()));
          /*
           *  Make sure we don't move back and forth,
           *  even if it's technically on a different edge
           *  (but the edge is geometrically the same as
           *  the current).
           */
          final InferredEdge thisCurrentEdge =
              newEdge.getInferredEdge();
          final boolean found = Iterables.removeIf(transferEdges,
              new Predicate<InferredEdge>() {
                @Override
                public boolean apply(
                  @Nullable InferredEdge input) {
                  return input.getGeometry().equalsExact(
                      thisCurrentEdge.getGeometry());
                }
              });

          if (transferEdges.isEmpty()) {
            /*
             * We've got nowhere else to go, so stop here.
             */
            newStateDist.getMean().setElement(0, initialLocation
                + distTraveled);
            newStateDist.getMean().setElement(1, 0d);
            _log.warn("No on-road transfer edges to sample: movementError="
                + (totalDistToTravel - distTraveled));
            break;
          }
        }
      }

      final InferredEdge sampledEdge =
          edgeTransDist.sample(
              rng,
              transferEdges,
              newEdge == null ? currentStateBelief.getEdge()
                  .getInferredEdge() : newEdge
                  .getInferredEdge());

      if (sampledEdge.isNullEdge()) {

        if (totalDistToTravel != null) {
          /*
           * XXX TODO We don't allow this anymore.  Too complicated for now.
           */
          throw new IllegalStateException(
              "unsupported off-road movement");
        }

        /*
         * Project like a normal Kalman filter, but first we
         * have to convert road states to ground coordinates. 
         * Notice that we use the belief, since we want to
         * convert the covariance as well.
         */
        if (!newEdge.isNullEdge()) {
          newStateDist = PathUtils.getGroundBeliefFromRoad(
              newStateDist, newEdge, true);
        }
        
        final Vector projectedMean =
            movementFilter.getGroundModel().getA()
                .times(newStateDist.getMean());
        
        /*
         * If we were flying toward the edge of the graph,
         * such that we'll end up flying off of it, make a 
         * dead stop 
         */
        Envelope graphExtent = this.inferenceGraph.getProjGraphExtent();
        if (!newEdge.isNullEdge() &&
            !graphExtent.isNull() && !graphExtent.contains(
             GeoUtils.getCoordinates(AbstractRoadTrackingFilter.getOg().
                 times(projectedMean)))) {
          projectedMean.setElement(1, 0d);
          projectedMean.setElement(3, 0d);
          _log.warn("Graph extent reached.  State velocity zero'ed");
        }

        /*
         * Add some transition noise.
         */
//        final Vector noisyStateTrans = sampleStateTransInsideGraph(projectedMean, 
//            movementFilter, rng);
        final Vector noisyStateTrans = movementFilter.
            sampleStateTransDist(projectedMean, rng);
        newStateDist.setMean(noisyStateTrans);

        newEdge = this.inferenceGraph.getNullPathEdge();
        newPathEdges.add(this.inferenceGraph.getNullPathEdge());
        break;
      }

      /*
       * If we're here, then we're moving along edges.
       */

      if (totalDistToTravel == null) {
        /*
         * Predict the movement, i.e. distance and direction to travel. The mean
         * of this belief should be set to the true value, so the prediction is
         * exact.
         * 
         * Note: sampledEdge is the start edge in this case.
         */
        if (newStateDist.getMean().getDimensionality() == 4) {
          newStateDist = PathUtils.getRoadBeliefFromGround(
              newStateDist, sampledEdge, true);
        } else {
          /*
           * In this case, we were on-road and still are.
           * However, it is possible that we've sampled a
           * topologically equivalent edge to our source edge,
           * so we must adjust the state to reflect the new edge.
           */
          final InferredPath edgePath =
              this.inferenceGraph.getInferredPath(this.inferenceGraph
                  .getPathEdge(sampledEdge, 0d, 
                      currentStateBelief.getPath().isBackward()));
          newStateDist.setMean(
              edgePath.getStateOnPath(currentStateBelief)
              .getGlobalState());
        }

        initialLocation = newStateDist.getMean().getElement(0);

        final Vector projectedMean =
            movementFilter.getRoadModel().getA()
                .times(newStateDist.getMean());

        /*
         * Add some transition noise.
         */
        //        final Matrix sampleCovChol = StatisticsUtil.getCholR(movementFilter.getOnRoadStateTransCovar());
        //        newState = MultivariateGaussian.sample(projectedMean, sampleCovChol.transpose(), rng);
        newStateDist.setMean(
            movementFilter.sampleStateTransDist(
                projectedMean, rng));
        
        totalDistToTravel =
            newStateDist.getMean().getElement(0) - initialLocation;

        double newLocation = newStateDist.getMean().getElement(0);

        /*
         * Now that we know where we're going, set the directional edge
         */
        newEdge =
            this.inferenceGraph.getPathEdge(sampledEdge, 0d,
                newLocation < 0d);

        final double L =
            newEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && initialLocation > 0d) {
          initialLocation = -L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newStateDist.getMean().setElement(0, newLocation);
        } else if (newLocation >= 0d
            && initialLocation < 0d) {
          initialLocation = L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newStateDist.getMean().setElement(0, newLocation);
        }

        /*
         * Get the distance we've covered to move off of this edge, if we
         * have moved off.
         */
        final double direction =
            totalDistToTravel >= 0d ? 1d : -1d;
        if (L < Math.abs(newLocation)) {
          final double r = L - Math.abs(initialLocation);
          distTraveled += r * direction;
        } else {
          distTraveled += totalDistToTravel;
        }
      } else {

        final double direction =
            newStateDist.getMean().getElement(0) >= 0d ? 1d : -1d;
        final double sampledPathEdgeDistToStart =
            lastNewEdge == null
                || lastNewEdge.isNullEdge() ? 0d
                : direction
                    * lastNewEdge.getInferredEdge()
                        .getLength()
                    + lastNewEdge.getDistToStartOfEdge();

        final PathEdge sampledPathEdge =
            this.inferenceGraph.getPathEdge(sampledEdge,
                sampledPathEdgeDistToStart, direction < 0d);

        /*
         * Continue along edges
         */
        distTraveled +=
            direction
                * sampledPathEdge.getInferredEdge()
                    .getLength();
        newEdge = sampledPathEdge;
      }
      lastNewEdge = newEdge;
      newPathEdges.add(newEdge);
    }

    assert Preconditions.checkNotNull(
      (Iterables.getLast(newPathEdges).isNullEdge() || Iterables
          .getLast(newPathEdges).isOnEdge(
              newStateDist.getMean().getElement(0))) ? Boolean.TRUE : null);

    final PathStateBelief result;
    if (newStateDist.getInputDimensionality() == 2) {
      final InferredPath newPath =
          this.inferenceGraph.getInferredPath(newPathEdges,
              newStateDist.getMean().getElement(0) < 0d ? true : false);
      result =
          Preconditions.checkNotNull(newPath
              .getStateBeliefOnPath(newStateDist));
    } else {
      final InferredPath newPath =
          this.inferenceGraph.getNullPath();
      result = newPath.getStateBeliefOnPath(newStateDist);
    }

    return result;
  }

  /**
   * This method resamples the transition state until
   * it finds one that won't shoot the next state outside
   * of the graph bounds.
   * It has a fixed amount of tries before it throws a 
   * runtime error.
   * FIXME: this doesn't work when a lone edge travels
   * right into the boundary of the graph extent.  we
   * would have to bound the speed along the edge heading
   * toward the boundary. 
   *  
   * @param state
   * @param filter
   * @param rng
   * @return
   */
//  private Vector sampleStateTransInsideGraph(Vector state,
//    AbstractRoadTrackingFilter filter, Random rng) {
//    final int dim = state.getDimensionality();
//    final Matrix sampleCovChol =
//        StatisticsUtil.rootOfSemiDefinite(dim == 4 ? 
//            filter.getQg() : filter.getQr());
//    final Matrix covFactor = filter.getCovarianceFactor(dim == 2);
//    Envelope graphExtent = this.inferenceGraph.getProjGraphExtent();
//    Vector stateSmpl;
//    Coordinate newLoc;
//    int tries = 0;
//    do {
//      final Vector qSmpl =
//          MultivariateGaussian.sample(VectorFactory
//              .getDenseDefault().createVector(dim / 2),
//              sampleCovChol, rng);
//      
//      final Vector error = covFactor.times(qSmpl);
//      
//      stateSmpl = state.plus(error);
//      
//      newLoc =
//         GeoUtils.getCoordinates(AbstractRoadTrackingFilter.getOg().
//             times(stateSmpl));
//      
//      tries++;
//      if (tries >= maxGraphBoundsResampleTries)
//        throw new RuntimeException("Could not sample a state within the graph bounds");
//    
//    } while (!graphExtent.isNull() && !graphExtent.contains(newLoc));
//    
//    if (tries > 1)
//      _log.info("Inside graph bounds resample tries = " + tries);
//    
//    return stateSmpl;
//  }

  @Override
  public VehicleState
      update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

  /**
   * This method first samples a projected location/velocity state (consistent
   * with its motion model) then, if it's a road state, samples edges until it
   * reaches its projected distance, or until it cannot move any further on the
   * graph. <br>
   * The final product is a valid sampled transition state.
   * 
   * @param previousState
   * @param obs
   * @return
   */
  public VehicleState update(VehicleState previousState,
    GpsObservation obs) {

    /*
     * Nothing to update in this filter, besides the sampled path
     * and state.
     */
    final OnOffEdgeTransDirMulti newTransDist =
        previousState.getEdgeTransitionDist().clone();
    final AbstractRoadTrackingFilter predictedFilter =
        previousState.getMovementFilter().clone();

    final Random rng = this.random;
    this.seed = rng.nextLong();

    final PathStateBelief newPathState =
        sampleNextState(previousState);

    final VehicleState newState =
        this.inferenceGraph.createVehicleState(obs,
            predictedFilter, newPathState, newTransDist,
            previousState);

    return newState;
  }

}