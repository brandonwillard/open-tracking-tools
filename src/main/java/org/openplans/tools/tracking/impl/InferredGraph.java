package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;

import com.google.common.base.Preconditions;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class InferredGraph {

  public static class InferredEdge implements
      Comparable<InferredEdge> {

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

    public static InferredEdge getEmptyEdge() {
      return InferredEdge.emptyEdge;
    }

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
          4.4d, 1d / Math.pow(30d, 2d), 1d / Math.pow(30d, 2d) + 1d,
          Math.pow(30d, 2d));
      this.velocityEstimator = new UnivariateGaussianMeanVarianceBayesianEstimator(
          velocityPrecisionDist);
    }

    @Override
    public int compareTo(InferredEdge o) {
      final CompareToBuilder comparator = new CompareToBuilder();
      comparator.append(
          this.endVertex.getLabel(), o.endVertex.getLabel());
      comparator.append(
          this.startVertex.getLabel(), o.startVertex.getLabel());

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
      final InferredEdge other = (InferredEdge) obj;
      if (endVertex == null) {
        if (other.endVertex != null) {
          return false;
        }
      } else if (!endVertex.equals(other.endVertex)) {
        return false;
      }
      if (startVertex == null) {
        if (other.startVertex != null) {
          return false;
        }
      } else if (!startVertex.equals(other.startVertex)) {
        return false;
      }
      return true;
    }

    public Coordinate getCenterPointCoord() {
      return this.posGeometry.getCentroid().getCoordinate();
    }
    
    public boolean isEmptyEdge() {
      return this == emptyEdge;
    }

    public Coordinate getCoordOnEdge(Vector obsPoint) {
      if (this == InferredEdge.emptyEdge)
        return null;
      final Coordinate revObsPoint = new Coordinate(
          obsPoint.getElement(1), obsPoint.getElement(0));
      final LinearLocation here = posLocationIndexedLine
          .project(revObsPoint);
      final Coordinate pointOnLine = posLocationIndexedLine
          .extractPoint(here);
      final Coordinate revOnLine = new Coordinate(
          pointOnLine.y, pointOnLine.x);
      return revOnLine;
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
      this.graph.getNarratedGraph();
      for (final Edge edge : OtpGraph
          .filterForStreetEdges(this.startVertex.getIncoming())) {
        if (graph.getGraph().getIdForEdge(edge) != null)
          result.add(graph.getInferredEdge(edge));
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
          + ((endVertex == null) ? 0 : endVertex.hashCode());
      result = prime * result
          + ((startVertex == null) ? 0 : startVertex.hashCode());
      return result;
    }

    @Override
    public String toString() {
      if (this == emptyEdge)
        return "InferredEdge [empty edge]";
      else
        return "InferredEdge [edgeId=" + edgeId + ", length="
            + length + "]";
    }

  }

  private static class PathKey {

    private final VehicleState state;
    private final Coordinate startCoord;
    private final Coordinate endCoord;
    private final double distanceToTravel;

    public PathKey(VehicleState state, Coordinate start,
      Coordinate end, double distance) {
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

    public Coordinate getEndCoord() {
      return endCoord;
    }

    public Coordinate getStartCoord() {
      return startCoord;
    }

    public VehicleState getState() {
      return state;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result
          + ((endCoord == null) ? 0 : endCoord.hashCode());
      result = prime * result
          + ((startCoord == null) ? 0 : startCoord.hashCode());
      return result;
    }

  }

  public static class VertexPair {

    private final Vertex startVertex;
    private final Vertex endVertex;

    public VertexPair(Vertex startVertex, Vertex endVertex) {
      this.startVertex = startVertex;
      this.endVertex = endVertex;
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
      final VertexPair other = (VertexPair) obj;
      if (endVertex == null) {
        if (other.endVertex != null) {
          return false;
        }
      } else if (!endVertex.equals(other.endVertex)) {
        return false;
      }
      if (startVertex == null) {
        if (other.startVertex != null) {
          return false;
        }
      } else if (!startVertex.equals(other.startVertex)) {
        return false;
      }
      return true;
    }

    public Vertex getEndVertex() {
      return endVertex;
    }

    public Vertex getStartVertex() {
      return startVertex;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result
          + ((endVertex == null) ? 0 : endVertex.hashCode());
      result = prime * result
          + ((startVertex == null) ? 0 : startVertex.hashCode());
      return result;
    }

  }

  private final Map<VertexPair, InferredEdge> edgeToInfo = Maps
      .newHashMap();

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

  private Set<InferredPath> computePaths(PathKey key) {

    /*
     * We always consider moving off of an edge, staying on an edge, and
     * whatever else we can find.
     */
    final InferredEdge currentEdge = key.getState().getInferredEdge();

    final Coordinate toCoord = GeoUtils.reverseCoordinates(key
        .getEndCoord());
    final Set<InferredPath> paths = Sets.newHashSet(InferredPath
        .getEmptyPath());
    final Set<Edge> startEdges = Sets.newHashSet();
    final double stateStdDevDistance = 1.98d * Math.sqrt(key
        .getState().getBelief().getCovariance().normFrobenius());
    if (!currentEdge.isEmptyEdge()) {

      startEdges.add(currentEdge.getEdge());

    } else {
      final Coordinate fromCoord = GeoUtils.reverseCoordinates(key
          .getStartCoord());
      final Envelope fromEnv = new Envelope(fromCoord);
      fromEnv.expandBy(GeoUtils
          .getMetersInAngleDegrees(stateStdDevDistance));
      for (final Object obj : this.pathSampler.getEdgeIndex().query(
          fromEnv)) {
        final Edge edge = (Edge) obj;
        startEdges.add(edge);
      }
    }

    final Set<Edge> endEdges = Sets.newHashSet();

    final Envelope toEnv = new Envelope(toCoord);
    final double obsStdDevDistance = 1.98d * Math.sqrt(key.getState()
        .getMovementFilter().getObsVariance().normFrobenius());
    toEnv.expandBy(GeoUtils
        .getMetersInAngleDegrees(obsStdDevDistance));

    for (final Object obj : this.pathSampler.getEdgeIndex().query(
        toEnv)) {
      final Edge edge = (Edge) obj;
      endEdges.add(edge);
    }

    final List<Edge> endEdgeList = Lists.newArrayList(endEdges);
    for (final Edge startEdge : startEdges) {
      final MultiDestinationAStar forwardAStar = new MultiDestinationAStar(
          this.graph, endEdgeList, toCoord, obsStdDevDistance,
          startEdge);
      final MultiDestinationAStar backwardAStar = new MultiDestinationAStar(
          this.graph, endEdgeList, toCoord, obsStdDevDistance,
          startEdge);

      final ShortestPathTree spt1 = forwardAStar.getSPT(false);
      final ShortestPathTree spt2 = backwardAStar.getSPT(true);

      for (final Edge endEdge : endEdgeList) {
        final GraphPath forwardPath = spt1.getPath(
            endEdge.getToVertex(), false);
        if (forwardPath != null) {
          final InferredPath forwardResult = copyAStarResults(
              forwardPath, startEdge, false);
          if (forwardResult != null)
            paths.add(forwardResult);
        }
        final GraphPath backwardPath = spt2.getPath(
            endEdge.getToVertex(), false);
        if (backwardPath != null) {
          final InferredPath backwardResult = copyAStarResults(
              backwardPath, startEdge, true);
          if (backwardResult != null)
            paths.add(backwardResult);
        }
      }
    }

    return paths;
  }

  private InferredPath copyAStarResults(GraphPath gpath,
    Edge startEdge, boolean isReverse) {
    final double direction = isReverse ? -1d : 1d;
    double pathDist = 0d;
    final List<PathEdge> path = Lists.newArrayList();
    final PathEdge startPathEdge = PathEdge.getEdge(this.getInferredEdge(startEdge));
    if (gpath.edges.isEmpty()) {
      path.add(startPathEdge);
    } else {
      for (final Edge edge : isReverse ? Lists.reverse(gpath.edges)
          : gpath.edges) {
        PathEdge pathEdge = getValidPathEdge(edge, pathDist, direction, path);
        pathDist += direction * pathEdge.getInferredEdge().getLength();
        path.add(pathEdge);
      }
    }
    if (!path.isEmpty())
      return InferredPath.getInferredPath(path);
    else
      return null;
  }

  private PathEdge getValidPathEdge(Edge edge, double pathDist, double direction, List<PathEdge> path) {
    if (OtpGraph.isStreetEdge(edge)
        && edge.getGeometry() != null
        && edge.getDistance() > 0d
        && graph.getIdForEdge(edge) != null
        && !edge.equals(Iterables.getLast(path, null))) {

      return PathEdge.getEdge(this.getInferredEdge(edge), pathDist);

    } else if (edge.getFromVertex() != null
        && !edge.getFromVertex().getOutgoingStreetEdges()
            .isEmpty()) {

      for (final Edge streetEdge : edge.getFromVertex()
          .getOutgoingStreetEdges()) {

        if (streetEdge.getGeometry() != null
            && !streetEdge.equals(Iterables.getLast(path, null))
            && streetEdge.getDistance() > 0d
            && graph.getIdForEdge(streetEdge) != null) {

          /*
           * Find a valid street edge to work with
           */
          return PathEdge.getEdge(this.getInferredEdge(streetEdge), pathDist);
        }
      }
    }
    
    return null;
  }
  
  public InferredEdge getEdge(int id) {

    final Edge edge = graph.getEdgeById(id);
    final VertexPair key = new VertexPair(
        edge.getFromVertex(), edge.getToVertex());
    InferredEdge edgeInfo = edgeToInfo.get(key);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, id, this);
      edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  public Graph getGraph() {
    return graph;
  }

  public InferredEdge getInferredEdge(Edge edge) {

    final VertexPair key = new VertexPair(
        edge.getFromVertex(), edge.getToVertex());
    InferredEdge edgeInfo = edgeToInfo.get(key);
    final Integer edgeId = graph.getIdForEdge(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, edgeId, this);
      edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  public OtpGraph getNarratedGraph() {
    return narratedGraph;
  }

  public List<StreetEdge> getNearbyEdges(
    MultivariateGaussian initialBelief,
    StandardRoadTrackingFilter trackingFilter) {
    Preconditions.checkArgument(initialBelief
        .getInputDimensionality() == 4);

    final Envelope toEnv = new Envelope(
        GeoUtils.convertToLonLat(StandardRoadTrackingFilter.getOg()
            .times(initialBelief.getMean())));
    final double varDistance = 1.98d * Math.sqrt(trackingFilter
        .getObsVariance().normFrobenius());
    toEnv.expandBy(GeoUtils.getMetersInAngleDegrees(varDistance));

    final List<StreetEdge> streetEdges = Lists.newArrayList();
    for (final Object obj : this.pathSampler.getEdgeIndex().query(
        toEnv)) {
      final Edge edge = (Edge) obj;
      streetEdges.add((StreetEdge) edge);
    }
    return streetEdges;
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
        .snapToGraph(latlon);
    for (final Edge edge : snappedEdges) {
      results.add(getInferredEdge(edge));
    }
    return results;
  }

  public Set<InferredPath> getPaths(VehicleState fromState,
    Coordinate toCoord) {
    Preconditions.checkNotNull(fromState);

    final Coordinate fromCoord;
    if (!fromState.getInferredEdge().isEmptyEdge()) {
      fromCoord = fromState.getInferredEdge().getCenterPointCoord();
    } else {
      fromCoord = GeoUtils.convertToLatLon(fromState
          .getMeanLocation());
    }

    final Set<InferredPath> paths = Sets.newHashSet();
    final PathKey startEndEntry = new PathKey(
        fromState, fromCoord, toCoord, 0d);
    paths.addAll(pathsCache.getUnchecked(startEndEntry));
    return paths;
  }

}
