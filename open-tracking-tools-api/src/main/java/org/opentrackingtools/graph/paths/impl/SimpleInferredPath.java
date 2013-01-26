package org.opentrackingtools.graph.paths.impl;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.EdgePredictiveResults;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathState;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.DataCube;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

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
public class SimpleInferredPath implements InferredPath {

  protected final ImmutableList<PathEdge> edges;
  protected final Double totalPathDistance;

  public List<String> edgeIds = Lists.newArrayList();

  /*
   * Note: single edges are considered forward
   */
  protected Boolean isBackward = null;

  protected final Geometry geometry;
  protected List<InferredEdge> normalEdges;

  protected static InferredPath nullPath =
      new SimpleInferredPath();

  protected SimpleInferredPath() {
    this.edges =
        ImmutableList.of((PathEdge)SimplePathEdge.getNullPathEdge());
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  protected SimpleInferredPath(ImmutableList<PathEdge> edges,
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
      
      if (!edge.isNullEdge()) {
        if (isBackward) {
          Preconditions
              .checkArgument(lastEdge == null
                  || lastEdge
                      .getInferredEdge()
                      .getStartPoint()
                      .equals(
                          edge.getInferredEdge()
                              .getEndPoint()));
        } else {
          Preconditions.checkArgument(lastEdge == null
              || lastEdge
                  .getInferredEdge()
                  .getEndPoint()
                  .equals(
                      edge.getInferredEdge()
                          .getStartPoint()));

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

  protected SimpleInferredPath(PathEdge edge) {
    Preconditions.checkArgument(!edge.isNullEdge());
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

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#clampToPath(double)
   */
  @Override
  public double clampToPath(final double distance) {
    final double dir = this.isBackward() ? -1d : 1d;
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
        o.getPathEdges().toArray());
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
    final SimpleInferredPath other = (SimpleInferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getCheckedStateOnPath(gov.sandia.cognition.math.matrix.Vector, double)
   */
  @Override
  public PathState getStateOnPath(Vector state,
    double tolerance) {
    Preconditions.checkArgument(tolerance >= 0d);
    Preconditions.checkState(!isNullPath() || state.getDimensionality() == 4);

    if (isNullPath()) {
      return SimplePathState.getPathState(this, state);
    }
    
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

    return SimplePathState.getPathState(this, newState);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getEdgeForDistance(double, boolean)
   */
  @Override
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

  public List<String> getEdgeIds() {
    return edgeIds;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getEdges()
   */
  @Override
  public ImmutableList<PathEdge> getPathEdges() {
    return edges;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getGeometry()
   */
  @Override
  public Geometry getGeometry() {
    return geometry;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getIsBackward()
   */
  @Override
  public Boolean isBackward() {
    return isBackward;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getPriorPredictionResults(org.opentrackingtools.impl.Observation, org.opentrackingtools.impl.VehicleState, java.util.Map)
   */
  @Override
  public
      InferredPathPrediction
      getPriorPredictionResults(
        GpsObservation obs,
        VehicleState state,
        Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik) {

    /*
     * We allow transitions from off-road onto a path, and vice-versa.  
     * Otherwise, we require that the first edge of the path is the edge of the
     * current state.
     */
    if (!(!state.getBelief().isOnRoad()
        || this.isNullPath() || state
        .getBelief()
        .getEdge()
        .getGeometry()
        .equalsTopo(
            Iterables.getFirst(this.getPathEdges(), null)
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
        && !this.isNullPath()) {
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
    final List<? extends PathEdge> edgesLocal = this.getPathEdges();
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

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getStateBeliefOnPath(org.opentrackingtools.graph.paths.states.impl.PathStateBelief)
   */
  @Override
  public SimplePathStateBelief getStateBeliefOnPath(
    PathStateBelief stateBelief) {
    final MultivariateGaussian edgeStateBelief;
    /*
     * Make sure it starts on this path.
     */
    if (!this.isNullPath()) {
      edgeStateBelief =
          stateBelief.getGlobalStateBelief().clone();
      /*
       * Note that we force projection onto the first edge
       * in this path.
       */
      if (!stateBelief.isOnRoad()) {
        AbstractRoadTrackingFilter
            .convertToRoadBelief(edgeStateBelief, this,
                Iterables.getFirst(this.getPathEdges(), null),
                true);
      } else {
        final Vector convertedState =
            this.getStateOnPath(stateBelief).getRawState();
        Preconditions.checkState(convertedState != null);
        edgeStateBelief.setMean(convertedState);
      }
    } else {
      edgeStateBelief = stateBelief.getGroundBelief();
    }

    return SimplePathStateBelief.getPathStateBelief(this,
        edgeStateBelief);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getStateOnPath(org.opentrackingtools.graph.paths.states.impl.AbstractPathState)
   */
  @Override
  public PathState getStateOnPath(
    PathState currentState) {

    Preconditions.checkState(!this.isNullPath());

    final SimplePathState startState =
        SimplePathState.getPathState(this, VectorFactory
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


    assert (this.isOnPath(normState.getElement(0)));
    assert Preconditions.checkNotNull(SimplePathState
        .getPathState(this, normState)
        .minus(currentState)
        .isZero(
            AbstractRoadTrackingFilter
                .getEdgeLengthErrorTolerance())
        ? Boolean.TRUE : null);

    return SimplePathState.getPathState(this, normState);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#getTotalPathDistance()
   */
  @Override
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

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#isEmptyPath()
   */
  @Override
  public boolean isNullPath() {
    return this == nullPath;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.impl.InferredPath#isOnPath(double)
   */
  @Override
  public boolean isOnPath(double distance) {

    Preconditions.checkState(!isNullPath());

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

  @Override
  public String toString() {
    if (this.isNullPath())
      return "SimpleInferredPath [null path]";
    else
      return "SimpleInferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance
          + "]";
  }

  public void updateEdges(GpsObservation obs,
    MultivariateGaussian stateBelief, InferenceGraph graph) {

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

  public static InferredPath getNullPath() {
    return nullPath;
  }

  public static SimpleInferredPath getInferredPath(
    List<? extends PathEdge> newEdges, boolean isBackward) {
    if (newEdges.size() == 1) {
      final PathEdge edge = Iterables.getOnlyElement(newEdges);
      if (edge.isNullEdge())
        return (SimpleInferredPath)nullPath;
    }
    return new SimpleInferredPath(ImmutableList.copyOf(newEdges),
        isBackward);
  }

  public static SimpleInferredPath getInferredPath(
    PathEdge pathEdge) {
    if (pathEdge.isNullEdge())
      return (SimpleInferredPath)nullPath;
    else
      return new SimpleInferredPath(pathEdge);
  }

  public List<InferredEdge> getInferredEdges() {
    return normalEdges;
  }

  @Override
  public PathStateBelief getStateBeliefOnPath(
    MultivariateGaussian belief) {
    return SimplePathStateBelief.getPathStateBelief(this, belief);
  }

  @Override
  public PathState getStateOnPath(Vector state) {
    return this.getStateOnPath(state, 
        AbstractRoadTrackingFilter.getEdgeLengthErrorTolerance());
  }

  @Override
  public InferredPath getPathTo(PathEdge edge) {
    
    final List<PathEdge> newEdges = Lists.newArrayList();
    for (PathEdge edge1 : this.getPathEdges()) {
      newEdges.add(edge1);
      if (edge1.equals(edge)) {
        break;
      }
    }
    
    InferredPath newPath = new SimpleInferredPath(
        ImmutableList.copyOf(newEdges), this.isBackward);
    
    return newPath;
  }

}
