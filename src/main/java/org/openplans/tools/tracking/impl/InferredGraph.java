package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.OutEdge;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.edgetype.TurnEdge;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.util.LengthConstrainedPathFinder;
import org.opentripplanner.routing.util.LengthConstrainedPathFinder.State;
import org.opentripplanner.routing.vertextype.TurnVertex;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableList.Builder;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class InferredGraph {

  public static class InferredEdge {

    private final Integer edgeId;
    private final Vertex startVertex;
    private final Vertex endVertex;
    private final Vector endPoint;
    private final Vector startPoint;
    private final double length;
    private final NormalInverseGammaDistribution velocityPrecisionDist;
    private final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;
    private final InferredGraph graph;

    private final Edge edge;

    private final Geometry posGeometry;
    private final LocationIndexedLine posLocationIndexedLine;
    private final LengthIndexedLine posLengthIndexedLine;
    private final LengthLocationMap posLengthLocationMap;

    // FIXME remove this negative junk; use turns.
    private final Geometry negGeometry;
    private final LocationIndexedLine negLocationIndexedLine;
    private final LengthIndexedLine negLengthIndexedLine;
    private final LengthLocationMap negLengthLocationMap;

    /*
     * This is the empty edge, which stands for free movement
     */
    private final static InferredGraph.InferredEdge emptyEdge = new InferredGraph.InferredEdge();

    private InferredEdge() {
      this.edgeId = null;
      this.endPoint = null;
      this.startPoint = null;
      this.length = 0;
      this.velocityEstimator = null;
      this.velocityPrecisionDist = null;
      this.startVertex = null;
      this.endVertex = null;
      this.graph = null;

      this.edge = null;

      this.posGeometry = null;
      this.posLocationIndexedLine = null;
      this.posLengthIndexedLine = null;
      this.posLengthLocationMap = null;

      this.negGeometry = null;
      this.negLocationIndexedLine = null;
      this.negLengthIndexedLine = null;
      this.negLengthLocationMap = null;
    }

    private InferredEdge(Edge edge, Integer edgeId,
      InferredGraph graph) {
      this.graph = graph;
      this.edgeId = edgeId;
      this.edge = edge;

      /*
       * Warning: this geometry is in lon/lat and may contain more than one
       * straight line.
       */
      this.posGeometry = edge.getGeometry();
      this.negGeometry = edge.getGeometry().reverse();

      this.posLocationIndexedLine = new LocationIndexedLine(
          posGeometry);
      this.posLengthIndexedLine = new LengthIndexedLine(posGeometry);
      this.posLengthLocationMap = new LengthLocationMap(posGeometry);

      this.negLocationIndexedLine = new LocationIndexedLine(
          negGeometry);
      this.negLengthIndexedLine = new LengthIndexedLine(negGeometry);
      this.negLengthLocationMap = new LengthLocationMap(negGeometry);

      this.startVertex = edge.getFromVertex();
      this.endVertex = edge.getToVertex();

      final Coordinate startPoint = this.posLocationIndexedLine
          .extractPoint(this.posLocationIndexedLine.getStartIndex());
      /*
       * We need to flip these coords around to get lat/lon.
       */
      final Coordinate startPointCoord = GeoUtils
          .convertToEuclidean(new Coordinate(
              startPoint.y, startPoint.x));
      this.startPoint = VectorFactory.getDefault().createVector2D(
          startPointCoord.x, startPointCoord.y);

      final Coordinate endPoint = this.posLocationIndexedLine
          .extractPoint(this.posLocationIndexedLine.getEndIndex());
      final Coordinate endPointCoord = GeoUtils
          .convertToEuclidean(new Coordinate(endPoint.y, endPoint.x));
      this.endPoint = VectorFactory.getDefault().createVector2D(
          endPointCoord.x, endPointCoord.y);

      this.length = GeoUtils.getAngleDegreesInMeters(posGeometry
          .getLength());

      this.velocityPrecisionDist =
      // ~4.4 m/s, std. dev ~ 30 m/s, Gamma with exp. value = 30 m/s
      // TODO perhaps variance of velocity should be in m/s^2. yeah...
      new NormalInverseGammaDistribution(
          266d, 1 / Math.sqrt(1800d), 1 / Math.sqrt(1800) + 1,
          Math.sqrt(1800));
      this.velocityEstimator = new UnivariateGaussianMeanVarianceBayesianEstimator(
          velocityPrecisionDist);
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
      final InferredEdge other = (InferredEdge) obj;
      if (posGeometry == null) {
        if (other.posGeometry != null) {
          return false;
        }
      } else if (!posGeometry.equals(other.posGeometry)) {
        return false;
      }
      return true;
    }

    public Coordinate getCenterPointCoord() {
      return this.posGeometry.getCentroid().getCoordinate();
    }

    public Edge getEdge() {
      return this.edge;
    }

    public Integer getEdgeId() {
      return edgeId;
    }

    public Vector getEndPoint() {
      return this.endPoint;
    }

    public Vertex getEndVertex() {
      return endVertex;
    }

    public InferredGraph getGraph() {
      return graph;
    }

    /**
     * This returns a list of edges that are incoming, wrt the direction of this
     * edge, and that are reachable from this edge (e.g. not one way in the
     * direction of this edge).
     * 
     * @return
     */
    public List<InferredEdge> getIncomingTransferableEdges() {

      final List<InferredEdge> result = Lists.newArrayList();
      for (final Edge edge : this.startVertex.getIncoming()) {

        final Set<Edge> tmpResults = Sets.newHashSet();
        if (edge.getGeometry() == null
            || !edge.getMode().equals(TraverseMode.CAR)) {
          /*
           * Only attempt one level of descent for finding street edges
           */
          tmpResults.addAll(edge.getFromVertex()
              .getOutgoingStreetEdges());
        } else {
          tmpResults.add(edge);
        }
        for (final Edge edge2 : tmpResults) {
          if (!edge2.getGeometry().equals(posGeometry))
            result.add(graph.getInferredEdge(edge2));
        }
      }

      return result;
    }

    public double getLength() {
      return length;
    }

    public Geometry getNegGeometry() {
      return negGeometry;
    }

    public LengthIndexedLine getNegLengthIndexedLine() {
      return negLengthIndexedLine;
    }

    public LengthLocationMap getNegLengthLocationMap() {
      return negLengthLocationMap;
    }

    public LocationIndexedLine getNegLocationIndexedLine() {
      return negLocationIndexedLine;
    }

    /**
     * This returns a list of edges that are outgoing, wrt the direction of this
     * edge, and that are reachable from this edge (e.g. not one way against the
     * direction of this edge).
     * 
     * @return
     */
    public List<InferredEdge> getOutgoingTransferableEdges() {
      final List<InferredEdge> result = Lists.newArrayList();
      for (final Edge edge : this.endVertex.getOutgoingStreetEdges()) {
        result.add(graph.getInferredEdge(edge));
      }

      return result;
    }

    /**
     * Get the snapped location in projected/euclidean coordinates for the given
     * obsPoint (in lat/lon).
     * 
     * @param obsPoint
     * @return
     */
    public Vector getPointOnEdge(Coordinate obsPoint) {
      if (this == InferredEdge.emptyEdge)
        return null;
      final Coordinate revObsPoint = new Coordinate(
          obsPoint.y, obsPoint.x);
      final LinearLocation here = posLocationIndexedLine
          .project(revObsPoint);
      final Coordinate pointOnLine = posLocationIndexedLine
          .extractPoint(here);
      final Coordinate revOnLine = new Coordinate(
          pointOnLine.y, pointOnLine.x);
      final Coordinate projPointOnLine = GeoUtils
          .convertToEuclidean(revOnLine);
      return VectorFactory.getDefault().createVector2D(
          projPointOnLine.x, projPointOnLine.y);
    }

    public Geometry getPosGeometry() {
      return posGeometry;
    }

    public LengthIndexedLine getPosLengthIndexedLine() {
      return posLengthIndexedLine;
    }

    public LengthLocationMap getPosLengthLocationMap() {
      return posLengthLocationMap;
    }

    public LocationIndexedLine getPosLocationIndexedLine() {
      return posLocationIndexedLine;
    }

    public Vector getStartPoint() {
      return startPoint;
    }

    public Vertex getStartVertex() {
      return startVertex;
    }

    public UnivariateGaussianMeanVarianceBayesianEstimator getVelocityEstimator() {
      return velocityEstimator;
    }

    public NormalInverseGammaDistribution getVelocityPrecisionDist() {
      return velocityPrecisionDist;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result
          + ((posGeometry == null) ? 0 : posGeometry.hashCode());
      return result;
    }

    @Override
    public String toString() {
      return "InferredEdge [edgeId=" + edgeId + ", endPoint="
          + endPoint + ", startPoint=" + startPoint + ", length="
          + length + "]";
    }

    public static InferredGraph.InferredEdge getEmptyEdge() {
      return emptyEdge;
    }

  }

  private static class PathKey {

    private final VehicleState state;
    private final Coordinate startCoord;
    private final Coordinate endCoord;
    private final double distanceToTravel;

    public PathKey(VehicleState state, Coordinate start, Coordinate end,
      double distance) {
      Preconditions.checkNotNull(state);
      Preconditions.checkNotNull(start);
      Preconditions.checkNotNull(end);
      this.state = state;
      this.startCoord = start;
      this.endCoord = end;
      this.distanceToTravel = distance;
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
      final PathKey other = (PathKey) obj;
      if (Double.doubleToLongBits(distanceToTravel) != Double
          .doubleToLongBits(other.distanceToTravel)) {
        return false;
      }
      if (endCoord == null) {
        if (other.endCoord != null) {
          return false;
        }
      } else if (!endCoord.equals(other.endCoord)) {
        return false;
      }
      if (startCoord == null) {
        if (other.startCoord != null) {
          return false;
        }
      } else if (!startCoord.equals(other.startCoord)) {
        return false;
      }
      return true;
    }

    public double getDistanceToTravel() {
      return distanceToTravel;
    }

    public VehicleState getState() {
      return state;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      long temp;
      temp = Double.doubleToLongBits(distanceToTravel);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      result = prime * result
          + ((endCoord == null) ? 0 : endCoord.hashCode());
      result = prime * result
          + ((startCoord == null) ? 0 : startCoord.hashCode());
      return result;
    }

    public Coordinate getStartCoord() {
      return startCoord;
    }

    public Coordinate getEndCoord() {
      return endCoord;
    }

  }

  private final Map<Edge, InferredEdge> edgeToInfo = Maps
      .newConcurrentMap();

  private final Graph graph;

  private final OtpGraph narratedGraph;

  private final PathSampler pathSampler;

  // private final LoadingCache<PathKey, Set<InferredPath>> pathsCache =
  // CacheBuilder.newBuilder()
  // .maximumSize(1000)
  // .build(
  // new CacheLoader<PathKey, Set<InferredPath>>() {
  // public Set<InferredPath> load(PathKey key) {
  // return computePaths(key);
  // }
  // });

  private final LoadingCache<PathKey, Set<InferredPath>> pathsCache = CacheBuilder
      .newBuilder().maximumSize(1000)
      .build(new CacheLoader<PathKey, Set<InferredPath>>() {
        @Override
        public Set<InferredPath> load(PathKey key) {
          return computePaths(key);
        }
      });

  public InferredGraph(OtpGraph graph) {
    this.graph = graph.getGraph();
    this.narratedGraph = graph;
    this.pathSampler = new PathSampler(graph.getGraph());
  }

//  private Set<InferredPath> computePaths(PathKey key) {
//    final Set<InferredPath> paths = Sets.newHashSet();
//
//    // TODO make sure the source edge by itself gets in the results
//    final LengthConstrainedPathFinder posFinder = new LengthConstrainedPathFinder(
//        key.getStartEdge(), key.getEndEdge(),
//        key.getDistanceToTravel(), 1e-7, true);
//
//    for (final State state : posFinder.getSolutions()) {
//      final List<PathEdge> pathEdges = Lists.newArrayList();
//      double currentDistance = 0d;
//      for (final Vertex vertex : state.toVertexList()) {
//        for (final Edge edge : vertex.getOutgoingStreetEdges()) {
//          pathEdges.add(PathEdge.getEdge(
//              getInferredEdge(edge), currentDistance));
//          currentDistance += edge.getDistance();
//        }
//      }
//      final InferredPath path = new InferredPath(
//          ImmutableList.copyOf(pathEdges));
//      paths.add(path);
//    }
//    
//    final LengthConstrainedPathFinder negFinder = new LengthConstrainedPathFinder(
//        key.getStartEdge(), key.getEndEdge(),
//        -key.getDistanceToTravel(), 1e-7, true);
//    
//    for (final State state : negFinder.getSolutions()) {
//      final List<PathEdge> pathEdges = Lists.newArrayList();
//      double currentDistance = 0d;
//      for (final Vertex vertex : state.toVertexList()) {
//        for (final Edge edge : vertex.getIncoming()) {
//          if (!(edge instanceof TurnEdge || edge instanceof OutEdge || edge instanceof PlainStreetEdge)) {
//              continue;
//          }
//          pathEdges.add(PathEdge.getEdge(
//              getInferredEdge(edge), currentDistance));
//          currentDistance += edge.getDistance();
//        }
//      }
//      final InferredPath path = new InferredPath(
//          ImmutableList.copyOf(pathEdges));
//      paths.add(path);
//    }
//
//    return paths;
//  }

  private Set<InferredPath> computePaths(PathKey key) {

    Coordinate fromCoord = GeoUtils.reverseCoordinates(key.getStartCoord());
    Coordinate toCoord = GeoUtils.reverseCoordinates(key.getEndCoord());

    /*
     * We always consider moving off of an edge, staying on an edge, and
     * whatever else we can find.
     */
    Set<InferredPath> paths = Sets.newHashSet(InferredPath
        .getEmptyPath());
    InferredEdge startEdge = key.getState().getInferredEdge();
    
    if (startEdge != InferredEdge.getEmptyEdge())
      paths.add(new InferredPath(ImmutableList.of(PathEdge
          .getEdge(startEdge))));
    
    Builder<PathEdge> path = ImmutableList.builder();
    final CoordinateSequence movementSeq = JTSFactoryFinder
        .getGeometryFactory().getCoordinateSequenceFactory()
        .create(new Coordinate[] { fromCoord, toCoord });

    final Geometry movementGeometry = JTSFactoryFinder
        .getGeometryFactory().createLineString(movementSeq);

    final double radiusAroundPoint = GeoUtils
        .getMetersInAngleDegrees(Math.sqrt(key.getState().getMovementFilter()
            .getGroundFilter().getMeasurementCovariance()
            .normFrobenius()));
    List<Edge> matcherResults = pathSampler.match(
        movementGeometry, radiusAroundPoint);
    
    final List<Edge> minimumConnectingEdges = Objects.firstNonNull(
        matcherResults, ImmutableList.<Edge> of());

    if (!minimumConnectingEdges.isEmpty()) {
      double pathDist = 0d;
      for (Edge pathEdge : minimumConnectingEdges) {
        path.add(PathEdge.getEdge(
            this.getInferredEdge(pathEdge), pathDist));
        pathDist += pathEdge.getDistance();
      }
      paths.add(new InferredPath(path.build(), pathDist));
    }
    return paths;
  }

  public InferredEdge getEdge(int id) {

    final Edge edge = graph.getEdgeById(id);
    InferredEdge edgeInfo = edgeToInfo.get(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, id, this);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }

  public Graph getGraph() {
    return graph;
  }

  public InferredEdge getInferredEdge(Edge edge) {

    InferredEdge edgeInfo = edgeToInfo.get(edge);
    final Integer edgeId = graph.getIdForEdge(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, edgeId, this);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }

  public OtpGraph getNarratedGraph() {
    return narratedGraph;
  }

  /**
   * Get nearby street edges from a projected point.
   * 
   * @param mean
   * @return
   */
  public Set<InferredEdge> getNearbyEdges(Vector mean) {
    final Set<InferredEdge> results = Sets.newHashSet();
    final Coordinate latlon = GeoUtils.convertToLatLon(mean);
    final List<StreetEdge> snappedEdges = this.narratedGraph
        .snapToGraph(null, latlon);
    for (final Edge edge : snappedEdges) {
      results.add(getInferredEdge(edge));
    }
    return results;
  }

  /**
   * Compute a path of InferredEdge's between observations. This includes the
   * empty edge. Also, it finds the edges nearest to the toCoord and gets paths
   * to the center points of those edges.
   * 
   * @param fromState
   * @param toCoord
   * @return
   */
//  public Set<InferredPath> getPathsNew(VehicleState fromState,
//    Coordinate toCoord) {
//    Preconditions.checkNotNull(fromState);
//
//    final Vector posVecTo = fromState.getNonVelocityVector();
//    final Vector posVecFrom = VehicleState
//        .getNonVelocityVector(fromState.getMovementFilter()
//            .getPrePredictiveBelief().getMean());
//    final Matrix O = fromState.getBelief().getInputDimensionality() == 4 ?
//        StandardRoadTrackingFilter.getOg() : StandardRoadTrackingFilter.getOr();
//    final double varDistance = 
//        O.times(fromState.getBelief().getCovariance()).times(O.transpose()).normFrobenius();
//        fromState.getBelief().getCovariance().normFrobenius();
//    final double distance = posVecFrom.euclideanDistance(posVecTo) 
//        + 1.98*Math.sqrt(varDistance);
//
//    Preconditions.checkArgument(distance < 1e4);
//
//    if (distance == 0d) {
//      final InferredPath path;
//      if (fromState.getInferredEdge() == InferredEdge.getEmptyEdge()) {
//        path = InferredPath.getEmptyPath();
//      } else {
//        path = new InferredPath(ImmutableList.of(PathEdge
//            .getEdge(fromState.getInferredEdge())));
//      }
//      return Sets.newHashSet(path);
//    }
//
//    final Set<Edge> startEdges = Sets.newHashSet();
//    if (fromState.getInferredEdge() != InferredEdge.getEmptyEdge()) {
//      startEdges.add(fromState.getInferredEdge().getEdge());
//    } else {
//      final Coordinate latlon = GeoUtils.convertToLatLon(fromState
//          .getMeanLocation());
//      startEdges.addAll(this.narratedGraph.snapToGraph(null, latlon));
//    }
//
//    final Set<StreetEdge> endEdges = Sets
//        .newHashSet(this.narratedGraph.snapToGraph(null, toCoord));
//    final Set<InferredPath> paths = Sets.newHashSet();
//    for (final Edge start : startEdges) {
//      for (final Edge end : endEdges) {
//        /*
//         * I'm adding the from and to edge lengths as a hack so that
//         * the path finder will not be too discriminant
//         */
//        final double distanceAdj = Math.max(distance, 
//            fromState.getMeanLocation().euclideanDistance(
//                fromState.getObservation().getProjectedPoint()))
//                + start.getDistance() + end.getDistance();
//        final PathKey startEndEntry = new PathKey(
//            fromState, start, end, distanceAdj);
//        paths.addAll(pathsCache.getUnchecked(startEndEntry));
//      }
//    }
//
//    paths.add(InferredPath.getEmptyPath());
//    return paths;
//  }
  
  public Set<InferredPath> getPaths(VehicleState fromState,
    Coordinate toCoord) {
    Preconditions.checkNotNull(fromState);
    
    final Set<InferredPath> paths = Sets.newHashSet();
    final PathKey startEndEntry = new PathKey(
        fromState, GeoUtils.convertToLatLon(fromState.getMeanLocation()), 
        toCoord, 0d);
    paths.addAll(pathsCache.getUnchecked(startEndEntry));
    return paths;
  }

  public static InferredEdge getEmptyEdge() {
    return InferredEdge.emptyEdge;
  }

}
