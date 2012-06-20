package org.openplans.tools.tracking.impl.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.algorithms.MultiDestinationAStar;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
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
import com.vividsolutions.jts.index.strtree.STRtree;

public class InferredGraph {

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
      for (final Object obj : this.narratedGraph.getEdgeIndex().query(
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

    for (final Object obj : this.narratedGraph.getEdgeIndex().query(
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
          if (forwardResult != null) {
            paths.add(forwardResult);
            // for debugging
            forwardResult.setStartEdge(this.getInferredEdge(startEdge));
            forwardResult.setEndEdge(this.getInferredEdge(endEdge));
          }
        }
        final GraphPath backwardPath = spt2.getPath(
            endEdge.getFromVertex(), false);
        if (backwardPath != null) {
          final InferredPath backwardResult = copyAStarResults(
              backwardPath, startEdge, true);
          if (backwardResult != null) {
            paths.add(backwardResult);
            // for debugging
            backwardResult.setStartEdge(this.getInferredEdge(startEdge));
            backwardResult.setEndEdge(this.getInferredEdge(endEdge));
          }
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

    if (edgeInfo == null) {
      final Integer edgeId = graph.getIdForEdge(edge);
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
    for (final Object obj : this.narratedGraph.getEdgeIndex().query(
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
