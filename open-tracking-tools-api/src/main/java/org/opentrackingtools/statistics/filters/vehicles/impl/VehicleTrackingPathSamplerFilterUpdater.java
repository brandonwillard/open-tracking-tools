package org.opentrackingtools.statistics.filters.vehicles.impl;

import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.apache.commons.lang.NotImplementedException;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;
import org.opentripplanner.routing.edgetype.StreetEdge;

import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;

public class VehicleTrackingPathSamplerFilterUpdater extends
    AbstractVTParticleFilterUpdater {

  private static final long serialVersionUID =
      2884138088944317656L;

  public VehicleTrackingPathSamplerFilterUpdater(
    GpsObservation obs, InferenceGraph inferenceGraph,
    VehicleStateInitialParameters parameters) {
    super(obs, inferenceGraph, parameters);
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

  @Override
  @Nonnull
  protected AbstractRoadTrackingFilter
      createRoadTrackingFilter() {
    return new StandardRoadTrackingFilter(
        parameters.getObsCov(),
        parameters.getOffRoadStateCov(),
        parameters.getOnRoadStateCov(),
        parameters.getInitialObsFreq());
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
      transferEdges.addAll(
          this.inferenceGraph.getIncomingTransferableEdges(inferredEdge));
    } else if (newState.getElement(0) > 0d) {
      transferEdges.addAll(this.inferenceGraph
          .getOutgoingTransferableEdges(inferredEdge));
    } else {
      transferEdges.addAll(this.inferenceGraph
          .getIncomingTransferableEdges(inferredEdge));
      transferEdges.addAll(this.inferenceGraph
          .getOutgoingTransferableEdges(inferredEdge));
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
  public PathStateBelief sampleNextState(
    OnOffEdgeTransDirMulti edgeTransDist,
    PathStateBelief pathStateBelief,
    AbstractRoadTrackingFilter<?> movementFilter) {

    /*
     * TODO FIXME there should probably be a flag for this debug stuff
     */
    final Random rng = this.random;
    rng.setSeed(this.seed);

    PathEdge currentEdge =
        this.inferenceGraph.getPathEdge(pathStateBelief.getEdge()
            .getInferredEdge(), 0d, pathStateBelief.getPath()
            .isBackward());
    PathEdge previousEdge = null;
    MultivariateGaussian newState = pathStateBelief.getLocalStateBelief().clone();

    final List<PathEdge> currentPath = Lists.newArrayList();

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
      if (currentEdge.isNullEdge()) {
        final Vector projLocation =
            AbstractRoadTrackingFilter.getOg().times(
                newState.getMean());
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
          transferEdges.add(pathStateBelief.getEdge().getInferredEdge());

          /*
           * We only allow going off-road when starting a path.
           */
          transferEdges.add(this.inferenceGraph.getNullInferredEdge());
        } else {
          transferEdges.addAll(getTransferableEdges(
              currentEdge.getInferredEdge(), newState.getMean()));
          /*
           *  Make sure we don't move back and forth,
           *  even if it's on a different edge.
           */
          final InferredEdge thisCurrentEdge =
              currentEdge.getInferredEdge();
          Iterables.removeIf(transferEdges,
              new Predicate<InferredEdge>() {
                @Override
                public boolean apply(
                  @Nullable InferredEdge input) {
                  return input.getGeometry().equalsTopo(
                      thisCurrentEdge.getGeometry());
                }
              });

          if (transferEdges.isEmpty()) {
            /*
             * We've got nowhere else to go, so stop here.
             */
            newState.getMean().setElement(0, initialLocation
                + distTraveled);
            newState.getMean().setElement(1, 0d);
            break;
          }
        }
      }

      final InferredEdge sampledEdge =
          edgeTransDist.sample(
              rng,
              transferEdges,
              currentEdge == null ? pathStateBelief.getEdge()
                  .getInferredEdge() : currentEdge
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
        if (!currentEdge.isNullEdge()) {
          PathUtils.convertToGroundBelief(
              newState, currentEdge, false, true);
          
          /*
           * When going from on road to off, we don't want to always
           * project along the edge length, so we're going to project
           * orthogonally to our road.
           * XXX: should remove this when/if using the bootstrap filter in
           * production, or on real data.
           */
          final double xVel = newState.getMean().getElement(1);
          final double yVel = newState.getMean().getElement(3);
          newState.getMean().setElement(1, yVel);
          newState.getMean().setElement(3, xVel);
        }
        final Vector projectedMean =
            movementFilter.getGroundModel().getA()
                .times(newState.getMean());

        /*
         * Add some transition noise.
         */
        //        final Matrix sampleCovChol = StatisticsUtil.getCholR(movementFilter.getQg());
        //        final Vector qSmpl = MultivariateGaussian.sample(projectedMean, 
        //            sampleCovChol.transpose(), rng);
        //        newState = projectedMean.plus(movementFilter.getCovarianceFactor(false).times(qSmpl));
        newState.setMean(movementFilter.sampleStateTransDist(
                projectedMean, rng));

        /*
         * Adjust for graph bounds.
         */
        final Coordinate newLoc =
            new Coordinate(newState.getMean().getElement(0),
                newState.getMean().getElement(2));
        if (!this.inferenceGraph.getProjGraphExtent()
            .contains(newLoc)) {
          /*
           * We're outside the bounds, so truncate at the bound edge.
           * TODO FIXME: This is an abrupt stop, and most filters won't
           * handle it well.  For comparisons, it's fine, since both
           * filters will likely suffer, no?
           */
          final Vector oldLoc =
              AbstractRoadTrackingFilter.getOg().times(
                  pathStateBelief.getGroundState());
          final Polygon extent =
              JTS.toGeometry(this.inferenceGraph
                  .getProjGraphExtent());
          final Geometry intersection =
              extent.intersection(JTSFactoryFinder
                  .getGeometryFactory().createLineString(
                      new Coordinate[] {
                          GeoUtils.getCoordinates(oldLoc),
                          //               new Coordinate(startEdge.getEdge().getCenterPointCoord()), 
                          newLoc }));
          newState.getMean().setElement(0,
              intersection.getCoordinates()[1].x);
          newState.getMean().setElement(1, 0d);
          newState.getMean().setElement(2,
              intersection.getCoordinates()[1].y);
          newState.getMean().setElement(3, 0d);
        }

        currentEdge = this.inferenceGraph.getNullPathEdge();
        currentPath.add(this.inferenceGraph.getNullPathEdge());
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
        if (newState.getMean().getDimensionality() == 4) {
          newState = PathUtils.getRoadBeliefFromGround(
              newState, sampledEdge, true);
        } else {
          /*
           * In this case, we were on-road and still are.
           * However, it is possible that we've sampled a
           * topologically equivalent edge to our source edge,
           * so we must adjust the state to reflect the new edge.
           */
          final InferredPath edgePath =
              this.inferenceGraph.getInferredPath(this.inferenceGraph
                  .getPathEdge(sampledEdge, 0d, false));
          newState.setMean(
              edgePath.getStateOnPath(newState.getMean())
              .getGlobalState());
        }

        initialLocation = newState.getMean().getElement(0);

        final Vector projectedMean =
            movementFilter.getRoadModel().getA()
                .times(newState.getMean());

        /*
         * Add some transition noise.
         */
        //        final Matrix sampleCovChol = StatisticsUtil.getCholR(movementFilter.getOnRoadStateTransCovar());
        //        newState = MultivariateGaussian.sample(projectedMean, sampleCovChol.transpose(), rng);
        newState.setMean(
            movementFilter.sampleStateTransDist(
                projectedMean, rng));

        totalDistToTravel =
            newState.getMean().getElement(0) - initialLocation;

        double newLocation = newState.getMean().getElement(0);

        /*
         * Now that we know where we're going, set the directional edge
         */
        currentEdge =
            this.inferenceGraph.getPathEdge(sampledEdge, 0d,
                newLocation < 0d);

        final double L =
            currentEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && initialLocation > 0d) {
          initialLocation = -L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newState.getMean().setElement(0, newLocation);
        } else if (newLocation >= 0d
            && initialLocation < 0d) {
          initialLocation = L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newState.getMean().setElement(0, newLocation);
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
            newState.getMean().getElement(0) >= 0d ? 1d : -1d;
        final double sampledPathEdgeDistToStart =
            previousEdge == null
                || previousEdge.isNullEdge() ? 0d
                : direction
                    * previousEdge.getInferredEdge()
                        .getLength()
                    + previousEdge.getDistToStartOfEdge();

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
        currentEdge = sampledPathEdge;
      }
      previousEdge = currentEdge;
      currentPath.add(currentEdge);
    }

    assert (Iterables.getLast(currentPath).isNullEdge() || Iterables
        .getLast(currentPath).isOnEdge(
            newState.getMean().getElement(0)));

    final PathStateBelief result;
    if (newState.getInputDimensionality() == 2) {
      final InferredPath newPath =
          this.inferenceGraph.getInferredPath(currentPath,
              newState.getMean().getElement(0) < 0d ? true : false);
      result =
          Preconditions.checkNotNull(newPath
              .getStateBeliefOnPath(newState));
    } else {
      final InferredPath newPath =
          this.inferenceGraph.getNullPath();
      result = newPath.getStateBeliefOnPath(newState);
    }

    return result;
  }

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
    final AbstractRoadTrackingFilter<?> predictedFilter =
        previousState.getMovementFilter().clone();

    final PathStateBelief currentBelief =
        previousState.getBelief().clone();

    /*
     * TODO FIXME again, a flag for this debug?
     */
    final Random rng = this.random;
    this.seed = rng.nextLong();

    final PathStateBelief newPathState =
        sampleNextState(newTransDist, currentBelief,
            predictedFilter);

    /*
     * Need to create 
     */
    if (newPathState.isOnRoad()
        && !currentBelief.isOnRoad()) {
//      AbstractRoadTrackingFilter.convertToRoadBelief(
//          newBelief, newPathState.getPath(), true);
    } else if (newPathState.isOnRoad()
        && !currentBelief.isOnRoad()) {
//      AbstractRoadTrackingFilter.convertToGroundBelief(
//          newBelief, newPathState.getEdge(), true);
    }

    final VehicleState newState =
        new VehicleState(this.inferenceGraph, obs,
            predictedFilter, newPathState, newTransDist,
            previousState);

    return newState;
  }

}