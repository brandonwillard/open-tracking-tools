package org.opentrackingtools.impl.graph.paths;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.impl.graph.InferredEdge;
import org.opentrackingtools.impl.statistics.DataCube;
import org.opentrackingtools.impl.statistics.StatisticsUtil;
import org.opentrackingtools.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.opentrackingtools.util.OtpGraph;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class InferredPath implements
    Comparable<InferredPath> {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;

  public List<Integer> edgeIds = Lists.newArrayList();

  /*
   * Note: single edges are considered forward
   */
  private Boolean isBackward = null;

  private final Geometry geometry;
  private List<InferredEdge> normalEdges;

  private static InferredPath emptyPath =
      new InferredPath();

  private InferredPath() {
    this.edges =
        ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  private InferredPath(ImmutableList<PathEdge> edges,
    boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    Preconditions
        .checkState(Iterables.getFirst(edges, null)
            .getDistToStartOfEdge() == 0d);
    this.edges = edges;
    this.isBackward = isBackward;
    this.normalEdges = Lists.newArrayList();

    PathEdge lastEdge = null;
    //    double absTotalDistance = 0d;
    final List<Coordinate> coords = Lists.newArrayList();
    for (final PathEdge edge : edges) {
      this.normalEdges.add(edge.getInferredEdge());
      
      if (!edge.isEmptyEdge()) {
        if (isBackward) {
          Preconditions
              .checkArgument(lastEdge == null
                  || lastEdge
                      .getInferredEdge()
                      .getStartVertex()
                      .equals(
                          edge.getInferredEdge()
                              .getEndVertex()));
        } else {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge
                  .getInferredEdge()
                  .getEndVertex()
                  .equals(
                      edge.getInferredEdge()
                          .getStartVertex()));

        }

        final Geometry geom = edge.getGeometry();
        if (geom.getLength() > 1e-4) {
          final Coordinate[] theseCoords =
              isBackward ? geom.reverse().getCoordinates()
                  : geom.getCoordinates();
          final int startIdx = coords.size() == 0 ? 0 : 1;
          for (int i = startIdx; i < theseCoords.length; i++) {
            if (i == 0
                || !theseCoords[i].equals(coords.get(coords
                    .size() - 1)))
              coords.add(theseCoords[i]);
          }
          edgeIds.add(edge.getInferredEdge().getEdgeId());
        }
      }

      lastEdge = edge;
    }

    if (edges.size() > 1) {
      this.geometry =
          JTSFactoryFinder.getGeometryFactory()
              .createLineString(
                  coords.toArray(new Coordinate[coords
                      .size()]));
    } else {
      final Geometry edgeGeom =
          Iterables.getOnlyElement(edges).getGeometry();
      this.geometry =
          isBackward ? edgeGeom.reverse() : edgeGeom;
    }

    final double direction = isBackward ? -1d : 1d;
    this.totalPathDistance =
        direction * this.geometry.getLength();
  }

  /*
   * Dangerous, since it loses direction info.
   */
  //  private InferredPath(InferredEdge inferredEdge) {
  //    Preconditions.checkArgument(!inferredEdge.isEmptyEdge());
  //    this.edges =
  //        ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d, false));
  //    this.totalPathDistance = inferredEdge.getLength();
  //    this.isBackward = Boolean.FALSE;
  //    this.edgeIds.add(inferredEdge.getEdgeId());
  //    this.geometry = inferredEdge.getGeometry();
  //  }

  private InferredPath(PathEdge edge) {
    Preconditions.checkArgument(!edge.isEmptyEdge());
    Preconditions
        .checkArgument(edge.getDistToStartOfEdge() == 0d);
    this.edges = ImmutableList.of(edge);
    this.normalEdges = ImmutableList.of(edge.getInferredEdge());
    this.isBackward = edge.isBackward();
    this.totalPathDistance =
        (this.isBackward ? -1d : 1d)
            * edge.getInferredEdge().getLength();
    this.edgeIds.add(edge.getInferredEdge().getEdgeId());
    this.geometry =
        this.isBackward ? edge.getGeometry().reverse()
            : edge.getGeometry();
  }

  public double clampToPath(final double distance) {
    final double dir = this.getIsBackward() ? -1d : 1d;
    final LengthIndexedLine lil =
        new LengthIndexedLine(this.getGeometry());
    final double clampedIndex =
        dir * lil.clampIndex(dir * distance);
    return clampedIndex;
  }

  @Override
  public int compareTo(InferredPath o) {
    final CompareToBuilder comparator =
        new CompareToBuilder();
    comparator.append(this.edges.toArray(),
        o.edges.toArray());
    return comparator.toComparison();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final InferredPath other = (InferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  /**
   * Does the same as {@link #isOnPath(double)} except it returns the trimmed
   * location if it falls within allowable error, and works with/returns a state
   * vector.
   * 
   * @param distance
   * @return
   */
  public PathState getCheckedStateOnPath(Vector state,
    double tolerance) {
    Preconditions.checkState(!isEmptyPath());
    Preconditions.checkArgument(tolerance >= 0d);
    Preconditions
        .checkArgument(state.getDimensionality() == 2);

    final Vector newState = state.clone();
    final double distance = newState.getElement(0);
    final double direction = Math.signum(totalPathDistance);
    final double overTheEndDist =
        direction * distance - Math.abs(totalPathDistance);
    if (overTheEndDist > 0d) {
      if (overTheEndDist > tolerance) {
        return null;
      } else {
        newState.setElement(0, totalPathDistance);
      }
    } else if (direction * distance < 0d) {
      if (direction * distance < tolerance) {
        return null;
      } else {
        newState.setElement(0, direction * 0d);
      }
    }

    return PathState.getPathState(this, newState);
  }

  /**
   * Returns the farthest PathEdge that the given distance could correspond to.
   * The clamp option will clamp the distance to the beginning or end of the
   * path.
   * 
   * @param distance
   * @param clamp
   * @return
   */
  public PathEdge getEdgeForDistance(double distance,
    boolean clamp) {
    final double direction = Math.signum(totalPathDistance);
    if (direction * distance - Math.abs(totalPathDistance) > AbstractRoadTrackingFilter
        .getEdgeLengthErrorTolerance()) {
      return clamp ? Iterables.getLast(edges) : null;
    } else if (direction * distance < 0d) {
      return clamp ? Iterables.getFirst(edges, null) : null;
    }

    for (final PathEdge edge : edges.reverse()) {
      if (edge.isOnEdge(distance))
        return edge;
    }

    assert Preconditions.checkNotNull(null);

    return null;
  }

  public List<Integer> getEdgeIds() {
    return edgeIds;
  }

  public ImmutableList<PathEdge> getEdges() {
    return edges;
  }

  public Geometry getGeometry() {
    return geometry;
  }

  public Boolean getIsBackward() {
    return isBackward;
  }

  /**
   * XXX: the state must have a prior predictive mean.
   * 
   * @param obs
   * @param state
   * @param edgeToPreBeliefAndLogLik
   * @return
   */
  public
      InferredPathPrediction
      getPriorPredictionResults(
        Observation obs,
        VehicleState state,
        Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik) {

    /*
     * We allow transitions from off-road onto a path, and vice-versa.  
     * Otherwise, we require that the first edge of the path is the edge of the
     * current state.
     */
    if (!(!state.getBelief().isOnRoad()
        || this.isEmptyPath() || state
        .getBelief()
        .getEdge()
        .getGeometry()
        .equalsTopo(
            Iterables.getFirst(this.getEdges(), null)
                .getGeometry())))
      return null;


    final AbstractRoadTrackingFilter<?> filter =
        state.getMovementFilter();
    
    final PathStateBelief beliefPrediction =
              filter.predict(state.getBelief(), this);
    
    /*
     * XXX: testing movement restrictions
     */
    if (!state.getBelief().isOnRoad()
        && !this.isEmptyPath()) {
      /*
       * We just got on a road
       */
      final double direction = this.isBackward ? -1d : 1d;
      final boolean isCorrectDirection =  
          AbstractRoadTrackingFilter.getVr().dotProduct(
              beliefPrediction.getRawState()) * direction >= 0d;
              
      if (!isCorrectDirection)
        return null;
    }

    double pathLogLik = Double.NEGATIVE_INFINITY;

    final List<WrappedWeightedValue<PathEdge>> weightedPathEdges =
        Lists.newArrayList();
    final List<PathEdge> edgesLocal = this.getEdges();
    //Collections.singletonList(Iterables.getLast(this.getEdges()));
    for (final PathEdge edge : edgesLocal) {

      final EdgePredictiveResults edgeResults;

      if (edgeToPreBeliefAndLogLik.containsKey(edge)) {
        final EdgePredictiveResults otherEdgeResults =
            edgeToPreBeliefAndLogLik.get(edge);

        if (otherEdgeResults == null)
          continue;

        final PathStateBelief locationPrediction =
            otherEdgeResults.getLocationPrediction();

        if (locationPrediction == null)
          continue;

        /*
         * Although path edges may be the same, transitions
         * aren't guaranteed to be, especially for starting
         * edges (off-on, on-on would mistakenly be equal).
         */
        final double edgeTransLik =
            state.getEdgeTransitionDist().logEvaluate(
                state.getBelief().getEdge()
                    .getInferredEdge(),
                locationPrediction.getEdge()
                    .getInferredEdge());

        edgeResults =
            new EdgePredictiveResults(beliefPrediction,
                locationPrediction,
                otherEdgeResults
                    .getEdgePredMarginalLogLik(),
                edgeTransLik,
                otherEdgeResults.getMeasurementPredLogLik());
      } else {

        edgeResults =
            edge.getPredictiveLikelihoodResults(this,
                state, beliefPrediction, obs);

        edgeToPreBeliefAndLogLik.put(edge, edgeResults);
        
      }

      if (edgeResults.getLocationPrediction() == null)
        continue;

      weightedPathEdges
          .add(new WrappedWeightedValue<PathEdge>(edge,
              edgeResults.getTotalLogLik()));
      pathLogLik =
          LogMath.add(pathLogLik,
              edgeResults.getTotalLogLik());

    }

    /*
     * Total of zero likelihood
     */
    if (Double.isInfinite(pathLogLik))
      return null;

    assert !Double.isNaN(pathLogLik);

    return new InferredPathPrediction(this,
        edgeToPreBeliefAndLogLik, filter,
        weightedPathEdges, pathLogLik);
  }

  /**
   * TODO FIXME: make this a generic getStateOnPath (replace the other one).
   * 
   * @see {@link InferredPath#getStateOnPath(AbstractPathState)}
   * @param stateBelief
   * @return
   */
  public PathStateBelief getStateBeliefOnPath(
    PathStateBelief stateBelief) {
    final MultivariateGaussian edgeStateBelief;
    /*
     * Make sure it starts on this path.
     */
    if (!this.isEmptyPath()) {
      edgeStateBelief =
          stateBelief.getGlobalStateBelief().clone();
      /*
       * Note that we force projection onto the first edge
       * in this path.
       */
      if (!stateBelief.isOnRoad()) {
        AbstractRoadTrackingFilter
            .convertToRoadBelief(edgeStateBelief, this,
                Iterables.getFirst(this.getEdges(), null),
                true);
      } else {
        final Vector convertedState =
            this.getStateOnPath(stateBelief);
        Preconditions.checkState(convertedState != null);
        edgeStateBelief.setMean(convertedState);
      }
    } else {
      edgeStateBelief = stateBelief.getGroundBelief();
    }

    return PathStateBelief.getPathStateBelief(this,
        edgeStateBelief);
  }

  /**
   * Converts location component of the mean to a location on this path, if any. <br>
   * Basically, if the state isn't already defined on the path, then we check if
   * the state's edge is the opposite direction to the first edge on this path.
   * If so, we can convert the state to this path's direction.
   * 
   * @param beliefPrediction
   */
  public Vector getStateOnPath(
    AbstractPathState currentState) {

    Preconditions.checkState(!this.isEmptyPath());

    final PathState startState =
        PathState.getPathState(this, VectorFactory
            .getDefault().createVector2D(0d, 0d));

    final Vector diff = startState.minus(currentState);
    diff.negativeEquals();

    final Vector normState =
        VectorFactory.getDefault().createVector2D(
            this.clampToPath(diff.getElement(0)),
            diff.getElement(1));

    /*
     * Make sure that we're still on the same edge,
     * or that we had the same starting edge.
     * 
     */
    if (!Iterables
        .getFirst(this.edges, null)
        .getGeometry()
        .equalsTopo(
            Iterables.getFirst(this.edges, null)
                .getGeometry())) {
      boolean foundMatch = false;
      for (final PathEdge thisEdge : this.edges) {
        if (thisEdge.isOnEdge(normState.getElement(0))
            && currentState.getEdge().getGeometry()
                .equalsTopo(thisEdge.getGeometry())) {
          foundMatch = true;
          break;
        }
      }

      if (!foundMatch)
        return null;
    }

    //    PathState result = PathState.getPathState(
    //        this, diff.negative());

    //    final InferredEdge presentEdge = currentState.getEdge().getInferredEdge();
    //    final InferredEdge startEdge = edges.get(0).getInferredEdge();
    //    
    //    Preconditions.checkState(presentEdge.getGeometry()
    //        .equalsTopo(startEdge.getGeometry()));
    //    
    //    final Vector normState = currentState.getLocalState().clone();
    //    
    //
    //    final double currentLocation = normState.getElement(0);
    //    final double destDirection = this.isBackward ? -1d : 1d;
    //    final double srcDirection = currentState.getPath().isBackward ? -1d : 1d;
    //
    //    final boolean pathEdgeIsReverse =
    //        !presentEdge.getGeometry().equalsExact(
    //            startEdge.getGeometry());
    //    if (pathEdgeIsReverse) {
    //      if (destDirection * srcDirection == 1d) {
    //        normState.setElement(0,
    //            destDirection
    //                * (startEdge.getLength() - Math.abs(currentLocation)));
    //      } else {
    //        normState.setElement(0, -1d * currentLocation);
    //      }
    //      normState.setElement(1, -1d * normState.getElement(1));
    //    } else {
    //      if (destDirection * srcDirection == -1d) {
    //        normState.setElement(0, destDirection * startEdge.getLength()
    //            + currentLocation);
    //      }
    //    }

    assert (this.isOnPath(normState.getElement(0)));
    assert Preconditions.checkNotNull(PathState
        .getPathState(this, normState)
        .minus(currentState)
        .isZero(
            AbstractRoadTrackingFilter
                .getEdgeLengthErrorTolerance())
        ? Boolean.TRUE : null);

    return normState;
  }

  public Double getTotalPathDistance() {
    return totalPathDistance;
  }


  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + ((edges == null) ? 0 : edges.hashCode());
    return result;
  }

  public boolean isEmptyPath() {
    return this == emptyPath;
  }

  /**
   * Checks if the distance is on the path given some allowable error defined by
   * {@link AbstractRoadTrackingFilter#getEdgeLengthErrorTolerance()}.
   * 
   * @param distance
   * @return
   */
  public boolean isOnPath(double distance) {

    Preconditions.checkState(!isEmptyPath());

    final double direction = Math.signum(totalPathDistance);
    final double overTheEndDist =
        direction * distance - Math.abs(totalPathDistance);
    if (overTheEndDist > AbstractRoadTrackingFilter
        .getEdgeLengthErrorTolerance()) {
      return false;
    } else if (direction * distance < -AbstractRoadTrackingFilter
        .getEdgeLengthErrorTolerance()) {
      return false;
    }

    return true;
  }

  public void setIsBackward(Boolean isBackward) {
    this.isBackward = isBackward;
  }

  @Override
  public String toString() {
    if (this == emptyPath)
      return "InferredPath [empty path]";
    else
      return "InferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance
          + "]";
  }

  public void updateEdges(Observation obs,
    MultivariateGaussian stateBelief, OtpGraph graph) {

    if (this.isEmptyPath())
      return;

    final BayesianCredibleInterval ciInterval =
        BayesianCredibleInterval.compute(
            new UnivariateGaussian(stateBelief.getMean()
                .getElement(1), stateBelief.getCovariance()
                .getElement(1, 1)), 0.95);

    /*
     * If we could be stopped, then don't update this
     */
    if (ciInterval.withinInterval(0d))
      return;

    final double velocity =
        stateBelief.getMean().getElement(1);
    for (final PathEdge edge : this.getEdges()) {
      if (!edge.isEmptyEdge()) {
        edge.getInferredEdge()
            .getVelocityEstimator()
            .update(
                edge.getInferredEdge()
                    .getVelocityPrecisionDist(),
                Math.abs(velocity));

        final HashMap<String, Integer> attributes =
            new HashMap<String, Integer>();

        final Integer interval =
            Math.round(((obs.getTimestamp().getHours() * 60) + obs
                .getTimestamp().getMinutes())
                / DataCube.INTERVAL);

        attributes.put("interval", interval);
        attributes.put("edge", edge.getInferredEdge()
            .getEdgeId());

        graph.getDataCube().store(Math.abs(velocity),
            attributes);
      }
    }
  }

  /*
   * Again, dangerous because it loses path info.
   */
  //  public static InferredPath
  //      getInferredPath(InferredEdge inferredEdge) {
  //    if (inferredEdge.isEmptyEdge())
  //      return emptyPath;
  //    else
  //      return new InferredPath(inferredEdge);
  //  }

  public static InferredPath getEmptyPath() {
    return emptyPath;
  }

  public static InferredPath getInferredPath(
    Collection<PathEdge> edges, boolean isBackward) {
    if (edges.size() == 1) {
      final PathEdge edge = Iterables.getOnlyElement(edges);
      if (edge.isEmptyEdge())
        return emptyPath;
    }
    return new InferredPath(ImmutableList.copyOf(edges),
        isBackward);
  }

  public static InferredPath getInferredPath(
    PathEdge pathEdge) {
    if (pathEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(pathEdge);
  }

  public List<InferredEdge> getNormalEdges() {
    return normalEdges;
  }

  //  private PathEdge indexOfFromStart(Vector inputPt, MultivariateGaussian dist) {
  //    
  //    Preconditions.checkArgument(inputPt.getDimensionality() == 4);
  //    
  //    final LocationIndexedLine lilPath = new LocationIndexedLine(this.geometry);
  //    final Coordinate inputCoord = GeoUtils.getCoordinates(
  //        AbstractRoadTrackingFilter.getOg().times(inputPt));
  ////    final LinearLocation refIndex = LengthLocationMap.getLocation(
  ////        this.geometry, state.getGroundState().getElement(0));
  ////    final double refDistanceAlong = LengthLocationMap.getLength(this.geometry, refIndex);
  //    
  //    double minDistance = Double.MAX_VALUE;
  //    LinearLocation minLoc = null;
  //
  //    LineSegment seg = new LineSegment();
  //    for (LinearIterator it = new LinearIterator(this.geometry);
  //         it.hasNext(); it.next()) {
  //      if (! it.isEndOfLine()) {
  //        seg.p0 = it.getSegmentStart();
  //        seg.p1 = it.getSegmentEnd();
  //        
  //        double segFrac = seg.segmentFraction(inputCoord);
  //        int candidateComponentIndex = it.getComponentIndex();
  //        int candidateSegmentIndex = it.getVertexIndex();
  //        
  //        final LinearLocation loc = new LinearLocation(
  //            candidateComponentIndex, candidateSegmentIndex, 
  //            segFrac);
  //        
  ////        double segDistance = seg.distance(inputPt) 
  ////            + Math.abs(refDistanceAlong - segDistanceAlong);
  //        
  //        final Vector thisLocPnt = GeoUtils.getVector(lilPath.extractPoint(loc));
  //        final double segDistance = dist.getProbabilityFunction()
  //            .logEvaluate(thisLocPnt);
  //        
  //        if (segDistance > minDistance) {
  //          // otherwise, save this as new minimum
  //          minLoc = loc;
  //          minDistance = segDistance;
  //        }
  //      }
  //    }
  //    
  //    final double resDistanceAlong = LengthLocationMap
  //        .getLength(this.geometry, minLoc);
  //    final PathEdge result;
  //    if (minLoc != null) {
  //      result = this.getEdgeForDistance(resDistanceAlong, false);
  //    } else {
  //      result = null;
  //    } 
  //    return result;
  //  }

}
