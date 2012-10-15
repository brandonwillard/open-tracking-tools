package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
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
import org.openplans.tools.tracking.impl.util.GeoUtils;
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

  private Observation initialObservation;

  private OtpGraph inferredGraph;

  private VehicleStateInitialParameters parameters;

  private Random random;

  public long seed;

  public VehicleTrackingPathSamplerFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
  }

  @Override
  public VehicleTrackingPathSamplerFilterUpdater clone() {
    try {
      VehicleTrackingPathSamplerFilterUpdater clone = 
          (VehicleTrackingPathSamplerFilterUpdater) super.clone();
      clone.seed = seed;
      clone.inferredGraph = inferredGraph;
      clone.initialObservation = initialObservation;
      clone.parameters = (VehicleStateInitialParameters) clone.parameters.clone();
      clone.random = random;
      return clone;
    } catch (CloneNotSupportedException e) {
      e.printStackTrace();
    }
    return null;
  }

  /**
   * Why is this required?
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
        return particle.getProbabilityFunction().logEvaluate(
            observation);
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>();

    for (int i = 0; i < numParticles; i++) {
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
          
          /**
           * I hate doing this, but since these are state instances,
           * they don't have distributions, so no variances.
           * TODO FIXME: we should have a better design.  one that
           * take the above into account.
           */
          state.getBelief().setCovarianceInverse(
              MatrixFactory.getDiagonalDefault().createMatrix(
                  state.getBelief().getMean().getDimensionality(),
                  state.getBelief().getMean().getDimensionality()));
      
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
      
      /**
       * I hate doing this, but since these are state instances,
       * they don't have distributions, so no variances.
       * TODO FIXME: we should have a better design.  one that
       * take the above into account.
       */
      state.getBelief().setCovarianceInverse(
          MatrixFactory.getDiagonalDefault().createMatrix(
              state.getBelief().getMean().getDimensionality(),
              state.getBelief().getMean().getDimensionality()));

  
      final double lik =
          state.getProbabilityFunction().evaluate(initialObservation);
  
      initialDist.increment(state, lik);
      
      retDist.increment(initialDist.sample(this.random));
    }

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
   * Sample a path for the given parameters.<br>
   * The method is simple:<br>
   * <ul>
   * <li> For road-states, sample a transition error in order to tell the direction
   *  of movement.  Then we determine the possible transition edges (for ground-states
   *  these are the snapped roads). </li>
   * <li> Sample a transition edge.  This could change our state-type 
   * (ground, road).  Then project our state, and finish if we're in a ground-state.</li> 
   * <li> If we're here then we're in a road-state and we continue sampling edges
   * (excluding off-road transitions) until we've traveled the projected distance.</li>
   * </ul>
   * 
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  public PathState sampleNextState(
    OnOffEdgeTransDirMulti edgeTransDist,
    Vector state, PathEdge startEdge,
    AbstractRoadTrackingFilter<?> movementFilter) {

    /*
     * TODO FIXME there should probably be a flag for this debug stuff
     */
    final Random rng = this.random;
    rng.setSeed(this.seed);

    PathEdge currentEdge = startEdge;
    PathEdge previousEdge = null;
    Vector newState = state.clone();

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
      if (currentEdge.isEmptyEdge()) {
        final Vector projLocation =
            AbstractRoadTrackingFilter.getOg().times(newState);
        final double radius = StatisticsUtil.getLargeNormalCovRadius(
            (DenseMatrix) movementFilter.getObsCovar());
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
          if (newState.getElement(0) < 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
          } else if (newState.getElement(0) > 0d) {
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
            newState.setElement(0, initialLocation + distTraveled);
            newState.setElement(1, 0d);
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
         * Project like a normal Kalman filter, but first we
         * have to convert road states to ground coordinates. 
         * Notice that we use the belief, since we want to
         * convert the covariance as well.
         */
        if (!currentEdge.isEmptyEdge()) {
          newState = AbstractRoadTrackingFilter.convertToGroundState(newState, 
              currentEdge, true);
          /*
           * When going from on road to off, we don't want to always
           * project along the edge length, so we're going to project
           * orthogonally to our road.
           */
          final double xVel = newState.getElement(1);
          final double yVel = newState.getElement(3);
          newState.setElement(1, yVel);
          newState.setElement(3, xVel);
        }
        final Vector projectedMean = movementFilter.getGroundModel().getA().times(newState);
        
        /*
         * Add some transition noise.
         */
//        final Matrix sampleCovChol = StatisticsUtil.getCholR(movementFilter.getQg());
//        final Vector qSmpl = MultivariateGaussian.sample(projectedMean, 
//            sampleCovChol.transpose(), rng);
//        newState = projectedMean.plus(movementFilter.getCovarianceFactor(false).times(qSmpl));
        newState = movementFilter.sampleStateTransDist(projectedMean, rng);
        
        /*
         * Adjust for graph bounds.
         */
        final Coordinate newLoc = new Coordinate(newState.getElement(0),
            newState.getElement(2));
        if (!this.inferredGraph.getBaseGraph().getExtent().contains(newLoc)) {
          /*
           * We're outside the bounds, so truncate at the bound edge.
           * TODO FIXME: This is an abrupt stop, and most filters won't
           * handle it well.  For comparisons, it's fine, since both
           * filters will likely suffer, no?
           */
          final Vector oldLoc = AbstractRoadTrackingFilter.getOg().times(
              startEdge.isEmptyEdge() ? 
                     AbstractRoadTrackingFilter.convertToGroundState(state, startEdge, true)
                     : state);
          final Polygon extent = JTS.toGeometry(this.inferredGraph.getBaseGraph().getExtent());
          final Geometry intersection = extent.intersection(
              JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] {
                  GeoUtils.getCoordinates(oldLoc),
//               new Coordinate(startEdge.getEdge().getCenterPointCoord()), 
               newLoc 
              }));
          newState.setElement(0, intersection.getCoordinates()[1].x);
          newState.setElement(1, 0d);
          newState.setElement(2, intersection.getCoordinates()[1].y);
          newState.setElement(3, 0d);
        } 

        currentEdge = PathEdge.getEmptyPathEdge();
        currentPath.add(PathEdge.getEmptyPathEdge());
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
        if (newState.getDimensionality() == 4) {
          final InferredPath edgePath = InferredPath.getInferredPath(
              PathEdge.getEdge(sampledEdge, 0d, false));
          newState = AbstractRoadTrackingFilter.convertToRoadState(newState,
              edgePath, true).getState();
        } 
        
        initialLocation = newState.getElement(0);
        
        final Vector projectedMean = movementFilter.getRoadModel().getA().times(newState);
        
        /*
         * Add some transition noise.
         */
//        final Matrix sampleCovChol = StatisticsUtil.getCholR(movementFilter.getOnRoadStateTransCovar());
//        newState = MultivariateGaussian.sample(projectedMean, sampleCovChol.transpose(), rng);
        newState = movementFilter.sampleStateTransDist(projectedMean, rng);
        
        
        totalDistToTravel = newState.getElement(0) - initialLocation;

        double newLocation = newState.getElement(0);
        
        /*
         * Now that we know where we're going, set the directional edge
         */
        currentEdge = PathEdge.getEdge(sampledEdge, 0d, newLocation < 0d);
        
        final double L = currentEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && initialLocation > 0d) {
          initialLocation = -L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newState.setElement(0, newLocation);
        } else if (newLocation >= 0d && initialLocation < 0d) {
          initialLocation = L + initialLocation;
          newLocation = initialLocation + totalDistToTravel;
          newState.setElement(0, newLocation);
        }

        /*
         * Get the distance we've covered to move off of this edge, if we
         * have moved off.
         */
        final double direction = totalDistToTravel >= 0d ? 1d : -1d;
        if (L < Math.abs(newLocation)) {
          final double r = L - Math.abs(initialLocation);
          distTraveled += r * direction;
        } else {
          distTraveled += totalDistToTravel;
        }
      } else {      
        
        double direction =
            newState.getElement(0) >= 0d ? 1d : -1d;
        final double sampledPathEdgeDistToStart = 
            previousEdge == null || previousEdge.isEmptyEdge() ? 0d : 
              direction * previousEdge.getInferredEdge().getLength() 
                + previousEdge.getDistToStartOfEdge();
  
        if (sampledEdge == null) {
          /*-
           * We have nowhere else to go, but we're not moving off of an edge, so 
           * we call this a stop.
           */
          newState.setElement(0, currentEdge.getDistToStartOfEdge() +
              direction * currentEdge.getInferredEdge().getLength());
          newState.setElement(1, 0d);
          break;
        }
        
        final PathEdge sampledPathEdge =
            PathEdge.getEdge(sampledEdge, sampledPathEdgeDistToStart, direction < 0d);

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
            newState.getElement(0)));
    
    final PathState result;
    if (newState.getDimensionality() == 2) {
      final InferredPath newPath = InferredPath.getInferredPath(currentPath,
          newState.getElement(0) < 0d ? true : false);
      result = newPath.getCheckedStateOnPath(newState, AbstractRoadTrackingFilter.getEdgelengthtolerance());
    } else {
      final InferredPath newPath = InferredPath.getEmptyPath();
      result = PathState.getPathState(newPath, newState);
    }
    
    return result;
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

  /**
   * This method first samples a projected location/velocity state 
   * (consistent with its motion model) then, if it's a road state, 
   * samples edges until it reaches its projected distance, or 
   * until it cannot move any further on the graph. <br>
   * The final product is a valid sampled transition state.
   * 
   * @param previousState
   * @param obs
   * @return
   */
  public VehicleState update(VehicleState previousState,
    Observation obs) {
    final MultivariateGaussian currentLocBelief =
        previousState.getBelief().clone();
    final PathEdge currentPathEdge =
        PathEdge.getEdge(previousState.getInferredEdge(), 0d, previousState.getPath().getIsBackward());

    /*
     * Nothing to update in this filter, besides the sampled path
     * and state.
     */
    final OnOffEdgeTransDirMulti newTransDist =
        previousState.getEdgeTransitionDist().clone();
    final AbstractRoadTrackingFilter<?> predictedFilter =
        previousState.getMovementFilter().clone();

    final MultivariateGaussian predictiveBelief = currentLocBelief.clone();
    
    /*
     * TODO FIXME again, a flag for this debug?
     */
    final Random rng = this.random;
    this.seed = rng.nextLong();
    
    final PathState newPathState =
        sampleNextState(newTransDist, predictiveBelief.getMean(), currentPathEdge,
            predictedFilter);
    predictiveBelief.setMean(newPathState.getState());
    
    final InferredPath newPath = newPathState.getPath();

    final VehicleState newState =
        new VehicleState(this.inferredGraph, obs, predictedFilter,
            predictiveBelief, newTransDist, newPath, previousState);

    return newState;
  }

}