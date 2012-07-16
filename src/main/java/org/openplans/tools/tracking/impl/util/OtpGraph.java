package org.openplans.tools.tracking.impl.util;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.netlib.blas.BLAS;
import org.openplans.tools.tracking.graph_builder.PlainStreetEdgeWithOSMData;
import org.openplans.tools.tracking.graph_builder.TurnVertexWithOSMData;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.BaseGraph;
import org.openplans.tools.tracking.impl.graph.CartesianDistanceLibrary;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.algorithms.MultiDestinationAStar;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
import org.opentripplanner.common.geometry.DistanceLibrary;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.edgetype.TurnEdge;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Graph.LoadLevel;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.GraphServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl.CandidateEdgeBundle;
import org.opentripplanner.routing.spt.GraphPath;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.ApplicationContext;
import org.springframework.context.support.GenericApplicationContext;

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
import com.vividsolutions.jts.index.strtree.STRtree;

public class OtpGraph {

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
      result =
          prime * result
              + ((endCoord == null) ? 0 : endCoord.hashCode());
      result =
          prime * result
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
      result =
          prime * result
              + ((endVertex == null) ? 0 : endVertex.hashCode());
      result =
          prime * result
              + ((startVertex == null) ? 0 : startVertex.hashCode());
      return result;
    }

  }

  private static final double MAX_DISTANCE_SPEED = 53.6448; // ~120 mph

  private static final Logger log = LoggerFactory
      .getLogger(OtpGraph.class);

  private final GraphServiceImpl gs;

  /**
   * This is the edge-based graph used in routing; neither it nor any edges form
   * it should ever be used outside of this class.
   */
  private final Graph turnGraph;

  /**
   * This is the original (intersection-and-street-segment) graph, used for all
   * inference tasks other than routing.
   */
  private final Graph baseGraph;
  //base index service is in projected coords
  private final StreetVertexIndexServiceImpl baseIndexService;
  private final static RoutingRequest defaultOptions =
      new RoutingRequest(TraverseMode.CAR);
  private final STRtree turnEdgeIndex = new STRtree();
  private final STRtree baseEdgeIndex = new STRtree();

  private final STRtree turnVertexIndex = new STRtree();

  private final DistanceLibrary distanceLibrary =
      new CartesianDistanceLibrary();

  private final Map<VertexPair, InferredEdge> edgeToInfo = Maps
      .newConcurrentMap();

  private final LoadingCache<PathKey, Set<InferredPath>> pathsCache =
      CacheBuilder.newBuilder().maximumSize(1000)
          .build(new CacheLoader<PathKey, Set<InferredPath>>() {
            @Override
            public Set<InferredPath> load(PathKey key) {
              return computeUniquePaths(key);
            }
          });

  public OtpGraph(String path) {
    log.info("Loading OTP graph...");
    log.info("Using BLAS: " + BLAS.getInstance().getClass().getName());
    gs = new GraphServiceImpl();
    gs.setLoadLevel(LoadLevel.DEBUG);

    final ApplicationContext appContext =
        new GenericApplicationContext();

    gs.setResourceLoader(appContext);

    gs.setPath(path);
    gs.refreshGraphs();

    turnGraph = gs.getGraph();
    baseGraph = turnGraph.getService(BaseGraph.class).getBaseGraph();

    baseIndexService =
        new StreetVertexIndexServiceImpl(baseGraph, distanceLibrary);
    createIndices(baseGraph, baseEdgeIndex, null);
    createIndices(turnGraph, turnEdgeIndex, turnVertexIndex);

    log.info("Graph loaded..");
  }

  private Set<InferredPath> computePaths(PathKey key) {

    /*
     * We always consider moving off of an edge, staying on an edge, and
     * whatever else we can find.
     */
    final InferredEdge currentEdge = key.getState().getInferredEdge();

    final Coordinate toCoord = key.getEndCoord();
    final Coordinate fromCoord = key.getStartCoord();

    final Set<InferredPath> paths =
        Sets.newHashSet(InferredPath.getEmptyPath());
    final Set<Edge> startEdges = Sets.newHashSet();

    if (!currentEdge.isEmptyEdge()) {

      final PlainStreetEdgeWithOSMData edge =
          (PlainStreetEdgeWithOSMData) currentEdge.getEdge();
      for (final Edge outgoing : edge.getTurnVertex().getOutgoing()) {
        startEdges.add(outgoing);
      }
    } else {
      final double stateStdDevDistance =
          1.98d * Math.sqrt(key.getState().getBelief()
              .getCovariance().normFrobenius()
              / Math.sqrt(key.getState().getBelief()
                  .getInputDimensionality()));
      for (final Object obj : getNearbyEdges(fromCoord,
          stateStdDevDistance)) {
        final PlainStreetEdgeWithOSMData edge =
            (PlainStreetEdgeWithOSMData) obj;
        startEdges.addAll(edge.getTurnVertex().getOutgoing());
      }
    }

    final Set<Edge> endEdges = Sets.newHashSet();

    final double obsStdDevDistance =
        key.getState().getMovementFilter()
            .getObservationErrorAbsRadius();
    //        1.98d * Math.sqrt(key.getState()
    //        .getMovementFilter().getObsVariance().normFrobenius()/Math.sqrt(2));

    for (final Object obj : getNearbyEdges(toCoord, obsStdDevDistance)) {
      final PlainStreetEdgeWithOSMData edge =
          (PlainStreetEdgeWithOSMData) obj;
      endEdges.addAll(edge.getTurnVertex().getOutgoing());
    }

    /*
     * If we're already on an edge, then we attempt to gauge how
     * far in the opposite direction we are willing to consider.
     */
    final double timeDiff =
        key.getState().getMovementFilter().getCurrentTimeDiff();
    final double distanceMax = MAX_DISTANCE_SPEED * timeDiff;

    for (final Edge startEdge : startEdges) {
      final MultiDestinationAStar forwardAStar =
          new MultiDestinationAStar(turnGraph, endEdges, toCoord,
              obsStdDevDistance, startEdge, distanceMax);

      final ShortestPathTree spt1 = forwardAStar.getSPT(false);

      final MultiDestinationAStar backwardAStar =
          new MultiDestinationAStar(turnGraph, endEdges, toCoord,
              obsStdDevDistance, startEdge, distanceMax);
      final ShortestPathTree spt2 = backwardAStar.getSPT(true);

      for (final Edge endEdge : endEdges) {
        final GraphPath forwardPath =
            spt1.getPath(endEdge.getToVertex(), false);
        if (forwardPath != null) {
          final InferredPath forwardResult =
              copyAStarResults(forwardPath, getBaseEdge(startEdge),
                  false);
          if (forwardResult != null) {
            paths.add(forwardResult);
            // for debugging
            forwardResult.setStartSearchEdge(this
                .getInferredEdge(startEdge));
            forwardResult.setEndSearchEdge(this
                .getInferredEdge(endEdge));
          }
        }

        if (backwardAStar != null) {
          final GraphPath backwardPath =
              spt2.getPath(endEdge.getFromVertex(), false);
          if (backwardPath != null) {
            final InferredPath backwardResult =
                copyAStarResults(backwardPath, startEdge, true);
            if (backwardResult != null) {
              paths.add(backwardResult);
              // for debugging
              backwardResult.setStartSearchEdge(this
                  .getInferredEdge(startEdge));
              backwardResult.setEndSearchEdge(this
                  .getInferredEdge(endEdge));
            }
          }
        }
      }
    }

    return paths;
  }

  private Set<InferredPath> computeUniquePaths(PathKey key) {
    final Set<InferredPath> paths = computePaths(key);
    makeUnique(paths);
    return paths;
  }

  private InferredPath copyAStarResults(GraphPath gpath,
    Edge startEdge, boolean isReverse) {
    final double direction = isReverse ? -1d : 1d;
    double pathDist = 0d;
    final List<PathEdge> path = Lists.newArrayList();
    final PathEdge startPathEdge =
        PathEdge.getEdge(this.getInferredEdge(startEdge));
    if (gpath.edges.isEmpty()) {
      path.add(startPathEdge);
    } else {
      for (final Edge edge : isReverse ? Lists.reverse(gpath.edges)
          : gpath.edges) {
        final PathEdge pathEdge =
            getValidPathEdge(edge, pathDist, direction, path);
        pathDist +=
            direction * pathEdge.getInferredEdge().getLength();
        path.add(pathEdge);
      }
    }
    if (!path.isEmpty())
      return InferredPath.getInferredPath(path, isReverse);
    else
      return null;
  }

  private void createIndices(Graph graph, STRtree edgeIndex,
    STRtree vertexIndex) {

    for (final Vertex v : graph.getVertices()) {
      if (vertexIndex != null) {
        final Envelope vertexEnvelope =
            new Envelope(v.getCoordinate());
        vertexIndex.insert(vertexEnvelope, v);
      }

      for (final Edge e : v.getOutgoing()) {
        if (graph.getIdForEdge(e) != null) {
          final Geometry geometry = e.getGeometry();
          final Envelope envelope = geometry.getEnvelopeInternal();
          edgeIndex.insert(envelope, e);
        }
      }
    }
  }

  private Edge getBaseEdge(Edge edge) {
    if (edge instanceof TurnEdge) {
      final TurnVertexWithOSMData base =
          (TurnVertexWithOSMData) edge.getFromVertex();
      edge = base.getOriginal();
    }
    return edge;
  }

  public Graph getBaseGraph() {
    return baseGraph;
  }

  //unused
  public InferredEdge getEdge(int id) {

    final Edge edge = baseGraph.getEdgeById(id);
    final VertexPair key =
        new VertexPair(edge.getFromVertex(), edge.getToVertex());
    InferredEdge edgeInfo = edgeToInfo.get(key);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, id, this);
      edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  public GraphServiceImpl getGs() {
    return gs;
  }

  public StreetVertexIndexServiceImpl getIndexService() {
    return baseIndexService;
  }

  public InferredEdge getInferredEdge(Edge edge) {
    edge = getBaseEdge(edge);

    final VertexPair key =
        new VertexPair(edge.getFromVertex(), edge.getToVertex());
    InferredEdge edgeInfo = edgeToInfo.get(key);

    if (edgeInfo == null) {
      final Integer edgeId = baseGraph.getIdForEdge(edge);
      edgeInfo = new InferredEdge(edge, edgeId, this);
      edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  public Collection<InferredEdge> getInferredEdges() {
    return edgeToInfo.values();
  }

  /**
   * Get nearby street edges from a projected point.
   * 
   * @param mean
   * @return
   */
  public Set<StreetEdge>
      getNearbyEdges(Coordinate loc, double radius) {

    final Envelope toEnv = new Envelope(loc);
    toEnv.expandBy(radius);
    final Set<StreetEdge> streetEdges = Sets.newHashSet();
    for (final Object obj : baseEdgeIndex.query(toEnv)) {
      if (((StreetEdge) obj).canTraverse(defaultOptions))
        streetEdges.add((StreetEdge) obj);
    }
    return streetEdges;
  }

  public List<StreetEdge> getNearbyEdges(
    MultivariateGaussian initialBelief,
    StandardRoadTrackingFilter trackingFilter) {
    Preconditions.checkArgument(initialBelief
        .getInputDimensionality() == 4);

    final Envelope toEnv =
        new Envelope(
            GeoUtils.makeCoordinate(StandardRoadTrackingFilter
                .getOg().times(initialBelief.getMean())));
    final double varDistance =
        trackingFilter.getObservationErrorAbsRadius();
    //        1.98d * Math.sqrt(trackingFilter
    //        .getObsVariance().normFrobenius()
    //        / Math.sqrt(initialBelief.getInputDimensionality()));
    toEnv.expandBy(varDistance);

    final List<StreetEdge> streetEdges = Lists.newArrayList();
    for (final Object obj : baseEdgeIndex.query(toEnv)) {
      final Edge edge = (Edge) obj;
      streetEdges.add((StreetEdge) edge);
    }
    return streetEdges;
  }

  public Set<StreetEdge> getNearbyEdges(Vector loc, double radius) {
    Preconditions.checkArgument(loc.getDimensionality() == 2);
    return getNearbyEdges(GeoUtils.makeCoordinate(loc), radius);
  }

  public RoutingRequest getOptions() {
    return defaultOptions;
  }

  public Set<InferredPath> getPaths(VehicleState fromState,
    Coordinate toCoord) {
    Preconditions.checkNotNull(fromState);

    final Coordinate fromCoord;
    if (!fromState.getInferredEdge().isEmptyEdge()) {
      fromCoord = fromState.getInferredEdge().getCenterPointCoord();
    } else {
      final Vector meanLocation = fromState.getMeanLocation();
      fromCoord =
          new Coordinate(meanLocation.getElement(0),
              meanLocation.getElement(1));
    }

    final Set<InferredPath> paths = Sets.newHashSet();
    final PathKey startEndEntry =
        new PathKey(fromState, fromCoord, toCoord, 0d);

    paths.addAll(pathsCache.getUnchecked(startEndEntry));
    //    paths.addAll(computeUniquePaths(startEndEntry));
    return paths;
  }

  private PathEdge getValidPathEdge(Edge originalEdge,
    double pathDist, double direction, List<PathEdge> path) {
    final Edge edge = getBaseEdge(originalEdge);
    if (OtpGraph.isStreetEdge(edge) && edge.getGeometry() != null
        && edge.getDistance() > 0d
        && baseGraph.getIdForEdge(edge) != null
        && !edge.equals(Iterables.getLast(path, null))) {

      return PathEdge.getEdge(this.getInferredEdge(edge), pathDist);

    } else if (edge.getFromVertex() != null
        && !edge.getFromVertex().getOutgoingStreetEdges().isEmpty()) {

      for (final Edge streetEdge : edge.getFromVertex()
          .getOutgoingStreetEdges()) {

        if (streetEdge.getGeometry() != null
            && !streetEdge.equals(Iterables.getLast(path, null))
            && streetEdge.getDistance() > 0d
            && baseGraph.getIdForEdge(streetEdge) != null) {

          /*
           * Find a valid street edge to work with
           */
          return PathEdge.getEdge(this.getInferredEdge(streetEdge),
              pathDist);
        }
      }
    }

    return null;
  }

  public int getVertexCount() {
    return baseGraph.getVertices().size();
  }

  public List<StreetEdge> snapToGraph(Coordinate toCoords) {

    Preconditions.checkNotNull(toCoords);

    final RoutingRequest options = OtpGraph.defaultOptions;
    final CandidateEdgeBundle edgeBundle =
        baseIndexService.getClosestEdges(toCoords, options, null,
            null);
    return edgeBundle.toEdgeList();
  }

  public static List<Edge>
      filterForStreetEdges(Collection<Edge> edges) {
    final List<Edge> result = Lists.newArrayList();
    for (final Edge out : edges) {
      if (!(out instanceof StreetEdge)) {
        continue;
      }
      result.add(out);
    }
    return result;
  }

  public static boolean isStreetEdge(Edge pathEdge) {
    if (!(pathEdge instanceof StreetEdge))
      return false;
    else
      return true;
  }

  /**
   * Assume that paths are a subset of a shortest path tree. Remove all paths
   * that are either duplicates or are subpaths of a longer path.
   * 
   * @param paths
   */
  public static void makeUnique(Set<InferredPath> paths) {
    final PathTree tree = new PathTree();

    final HashSet<InferredPath> toRemove =
        new HashSet<InferredPath>();
    for (final InferredPath path : paths) {
      PathTree cur = tree;
      for (final PathEdge edge : path.getEdges()) {
        cur = cur.apply(edge, path);
        if (cur.isLeaf()) {
          //we are visiting a node that was previously a leaf.  It is no longer a leaf
          //and the paths that had previously visited it should be removed.
          cur.isLeaf = false;
          toRemove.addAll(cur.paths);
          assert (cur.paths.size() == 1);
          cur.removePath(cur.paths.get(0));
        }
        cur.paths.add(path);
      }
      if (!cur.isLeaf) {
        //either this is a true internal node
        //or this is the first time we have visited it
        if (cur.children.size() == 0) {
          cur.isLeaf = true;
        } else {
          toRemove.add(path);
          cur.removePath(path);
        }
      }
    }
    paths.removeAll(toRemove);
  }

}
