package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nullable;

import org.apache.commons.lang.NotImplementedException;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;

public class VehicleTrackingPathSamplerFilterUpdater implements
    PLParticleFilterUpdater<Observation, VehicleState> {

  private static final long serialVersionUID = 2884138088944317656L;

  private final Observation initialObservation;

  private final OtpGraph inferredGraph;

  private final VehicleStateInitialParameters parameters;

  private Random random;

  public VehicleTrackingPathSamplerFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
  }

  @Override
  public VehicleTrackingPathSamplerFilterUpdater clone() {
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

    final StandardRoadTrackingFilter tmpTrackingFilter =
        new StandardRoadTrackingFilter(parameters.getObsCov(),
            parameters.getOffRoadStateCov(),
            parameters.getOnRoadStateCov(),
            parameters.getInitialObsFreq());
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

        final StandardRoadTrackingFilter trackingFilter =
            new StandardRoadTrackingFilter(
                parameters.getObsCov(),
                parameters.getOffRoadStateCov(),
                parameters.getOnRoadStateCov(),
                parameters.getInitialObsFreq());

        final OnOffEdgeTransDirMulti edgeTransDist =
            new OnOffEdgeTransDirMulti(inferredGraph,
                parameters.getOnTransitionProbs(),
                parameters.getOffTransitionProbs());

        final VehicleState state =
            new VehicleState(this.inferredGraph, initialObservation,
                pathEdge.getInferredEdge(), trackingFilter,
                edgeTransDist, this.random);

        final double lik =
            state.getProbabilityFunction().evaluate(
                initialObservation);

        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final StandardRoadTrackingFilter trackingFilter =
        new StandardRoadTrackingFilter(parameters.getObsCov(),
            parameters.getOffRoadStateCov(),
            parameters.getOnRoadStateCov(),
            parameters.getInitialObsFreq());

    final OnOffEdgeTransDirMulti edgeTransDist =
        new OnOffEdgeTransDirMulti(inferredGraph,
            parameters.getOnTransitionProbs(),
            parameters.getOffTransitionProbs());

    final VehicleState state =
        new VehicleState(this.inferredGraph, initialObservation,
            InferredEdge.getEmptyEdge(), trackingFilter,
            edgeTransDist, this.random);

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

  /**
   * Sample a path for the given parameters.
   * Note: the passed belief will be the resulting predictive belief.
   * 
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  public InferredPath traverseEdge(
    OnOffEdgeTransDirMulti edgeTransDist,
    final MultivariateGaussian belief, PathEdge startEdge,
    AbstractRoadTrackingFilter movementFilter) {

    /*
     * We project the road path
     */

    /*
     * TODO FIXME there should probably be a flag for this debug stuff
     */
    final Random rng = this.random;
    rng.setSeed(rng.nextLong());

    PathEdge currentEdge = startEdge;
    PathEdge previousEdge = null;
    final MultivariateGaussian newBelief = belief.clone();

    final List<PathEdge> currentPath = Lists.newArrayList();

    /*
     * This will signify the start distance on the
     * initial edge in the new direction of motion.
     */
    double initialLocation = 0d;
    double distTraveled = 0d;
    Double totalDistToTravel = null;
    while (totalDistToTravel == null ||
        Math.abs(totalDistToTravel) > Math.abs(distTraveled)) {

      final List<InferredEdge> transferEdges = Lists.newArrayList();
      if (currentEdge.getInferredEdge() == InferredEdge
          .getEmptyEdge()) {
        final Vector projLocation =
            AbstractRoadTrackingFilter.getOg().times(
                newBelief.getMean());
        final double radius = StatisticsUtil.getLargeNormalCovRadius(
            (DenseMatrix) movementFilter.getObsVariance());
        for (final StreetEdge edge : this.inferredGraph
            .getNearbyEdges(projLocation, radius)) {
          transferEdges.add(this.inferredGraph.getInferredEdge(edge));
        }
      } else {
        if (totalDistToTravel == null) {
          transferEdges.add(startEdge.getInferredEdge());
          /*
           * We only allow going off-road when starting a path.
           */
          transferEdges.add(InferredEdge.getEmptyEdge());
        } else {
          if (newBelief.getMean().getElement(0) < 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
          } else if (newBelief.getMean().getElement(0) > 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          } else {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          }
          /*
           *  Make sure we don't move back and forth,
           *  even if it's on a different edge.
           */
          final InferredEdge thisCurrentEdge = currentEdge.getInferredEdge();
          Iterables.removeIf(transferEdges, new Predicate<InferredEdge>() {
            @Override
            public boolean apply(@Nullable InferredEdge input) {
              return input.getGeometry().equalsTopo(thisCurrentEdge.getGeometry());
            }
          });
          
          if (transferEdges.isEmpty()) {
            /*
             * We've got nowhere else to go, so stop here.
             */
            newBelief.getMean().setElement(0, initialLocation + distTraveled);
            newBelief.getMean().setElement(1, 0d);
            break;
          }
        }
      }

      final InferredEdge sampledEdge =
          edgeTransDist.sample(rng, transferEdges,
              currentEdge == null ? startEdge.getInferredEdge()
                  : currentEdge.getInferredEdge());

      if (sampledEdge == InferredEdge.getEmptyEdge()) {

        if (totalDistToTravel != null) {
          /*
           * XXX TODO We don't allow this anymore.  Too complicated for now.
           */
          throw new IllegalStateException("unsupported off-road movement");
        }
              
        /*
         * Project like a normal Kalman filter 
         */
        movementFilter.predict(newBelief,
            PathEdge.getEmptyPathEdge(), startEdge);
        
        /*
         * Add some transition noise.
         */
        final Vector transStateSample =
            AbstractRoadTrackingFilter.sampleMovementBelief(rng,
                newBelief.getMean(), movementFilter);
        newBelief.setMean(transStateSample);
        
        /*
         * Adjust for graph bounds.
         */
        final Coordinate newLoc = new Coordinate(newBelief.getMean().getElement(0),
            newBelief.getMean().getElement(2));
        if (!this.inferredGraph.getBaseGraph().getExtent().contains(newLoc)) {
          /*
           * We're outside the bounds, so truncate at the bound edge.
           * TODO FIXME: This is an abrupt stop, and most filters won't
           * handle it well.  For comparisons, it's fine, since both
           * filters will likely suffer, no?
           */
          final Polygon extent = JTS.toGeometry(this.inferredGraph.getBaseGraph().getExtent());
          final Geometry intersection = extent.intersection(
              JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] {
               new Coordinate(startEdge.getEdge().getCenterPointCoord()), 
               newLoc 
              }));
          newBelief.getMean().setElement(0, intersection.getCoordinates()[1].x);
          newBelief.getMean().setElement(1, 0d);
          newBelief.getMean().setElement(2, intersection.getCoordinates()[1].y);
          newBelief.getMean().setElement(3, 0d);
        } 

        currentEdge = PathEdge.getEmptyPathEdge();
        currentPath.add(PathEdge.getEmptyPathEdge());
        break;
      }
      
      /*
       * If we're here, then we're moving along edges.
       */

      double direction =
          newBelief.getMean().getElement(0) >= 0d ? 1d : -1d;
      final double sampledPathEdgeDistToStart = 
          previousEdge == null || previousEdge.isEmptyEdge() ? 0d : 
            direction * previousEdge.getInferredEdge().getLength() 
              + previousEdge.getDistToStartOfEdge();
      final PathEdge sampledPathEdge =
          PathEdge.getEdge(sampledEdge, sampledPathEdgeDistToStart, direction < 0d);

      if (sampledPathEdge == null) {
        /*-
         * We have nowhere else to go, but we're not moving off of an edge, so 
         * we call this a stop.
         */
        newBelief.getMean().setElement(0,
            direction * currentEdge.getInferredEdge().getLength());
        newBelief.getMean().setElement(1, 0d);
        break;
      }

      if (totalDistToTravel == null) {
        /*
         * Predict the movement, i.e. distance and direction to travel. The mean
         * of this belief should be set to the true value, so the prediction is
         * exact.
         */

        /*
         * Since we might be just transferring onto an edge, check
         * first.
         */
        final PathEdge initialEdge =
            startEdge.isEmptyEdge() ? sampledPathEdge : startEdge;
        currentEdge = initialEdge;

        if (newBelief.getInputDimensionality() == 4) {
          AbstractRoadTrackingFilter.convertToRoadBelief(newBelief,
              InferredPath.getInferredPath(initialEdge));
        }

        initialLocation = newBelief.getMean().getElement(0);
        movementFilter.predict(newBelief, initialEdge, initialEdge);

        final Vector transStateSample =
            AbstractRoadTrackingFilter.sampleMovementBelief(rng,
                newBelief.getMean(), movementFilter);
        newBelief.setMean(transStateSample);
        totalDistToTravel =
            newBelief.getMean().getElement(0) - initialLocation;

        double newLocation = newBelief.getMean().getElement(0);
        final double L = initialEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && initialLocation > 0d) {
          initialLocation = -L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        } else if (newLocation >= 0d && initialLocation < 0d) {
          initialLocation = L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        }

        /*
         * Get the distance we've covered to move off of this edge, if we
         * have moved off.
         */
        direction = totalDistToTravel >= 0d ? 1d : -1d;
        if (L < Math.abs(newLocation)) {
          final double r = L - Math.abs(initialLocation);
          distTraveled += r * direction;
        } else {
          distTraveled += totalDistToTravel;
        }
      } else {
        /*
         * Continue along edges
         */
        distTraveled +=
            direction * sampledPathEdge.getInferredEdge().getLength();
        currentEdge = sampledPathEdge;
      }
      previousEdge = currentEdge;
      currentPath.add(currentEdge);
    }

    assert (Iterables.getLast(currentPath).isEmptyEdge() || Iterables
        .getLast(currentPath).isOnEdge(
            newBelief.getMean().getElement(0)));

    belief.setMean(newBelief.getMean());
    belief.setCovariance(newBelief.getCovariance());
    return InferredPath.getInferredPath(currentPath,
        (totalDistToTravel != null && totalDistToTravel < 0d) ? true
            : false);
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

  public VehicleState update(VehicleState previousState,
    Observation obs) {
    final MultivariateGaussian currentLocBelief =
        previousState.getBelief().clone();
    final PathEdge currentPathEdge =
        PathEdge.getEdge(previousState.getInferredEdge());

    /*
     * TODO FIXME again, a flag for this debug?
     */
    final Random rng = this.random;
    rng.setSeed(rng.nextLong());

    /*
     * Nothing to update in this filter, besides the sampled path
     * and state.
     */
    final OnOffEdgeTransDirMulti newTransDist =
        previousState.getEdgeTransitionDist().clone();
    final AbstractRoadTrackingFilter predictedFilter =
        previousState.getMovementFilter().clone();

    final MultivariateGaussian predictiveBelief = currentLocBelief.clone();
    final InferredPath newPath =
        traverseEdge(newTransDist, predictiveBelief, currentPathEdge,
            predictedFilter);
    
    final PathEdge newPathEdge =
        Iterables.getLast(newPath.getEdges());
    /*
     * We don't really need to keep the gaussian distributions,
     * however, we do, so we must construct a valid posterior state now.
     */
    final MultivariateGaussian newBelief = predictiveBelief.clone();
    if (newPathEdge.isEmptyEdge()) {
      newBelief.setCovariance(previousState.getMovementFilter().getGroundFilter().getModelCovariance());
    } else {
      newBelief.setCovariance(previousState.getMovementFilter().getRoadFilter().getModelCovariance());
    }

    final VehicleState newState =
        new VehicleState(this.inferredGraph, obs, predictedFilter,
            newBelief, newTransDist, newPath,
            previousState);

    return newState;
  }

}