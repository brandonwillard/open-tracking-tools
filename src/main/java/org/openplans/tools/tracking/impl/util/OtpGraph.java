package org.openplans.tools.tracking.impl.util;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.io.File;
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
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.algorithms.MultiDestinationAStar;
import org.openplans.tools.tracking.impl.statistics.DataCube;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
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
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
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
      new RoutingRequest(new TraverseModeSet(TraverseMode.CAR));

  /*
   * Maximum radius we're willing to search around a given
   * observation when snapping (for path search destination edges)
   */
  private static final double MAX_OBS_SNAP_RADIUS = 200d;

  /*
   * Maximum radius we're willing to search around a given
   * state when snapping (for path search off -> on-road edges)
   */
  private static final double MAX_STATE_SNAP_RADIUS = 350d;

  private final STRtree turnEdgeIndex = new STRtree();
  private final STRtree baseEdgeIndex = new STRtree();

  private final STRtree turnVertexIndex = new STRtree();
  private final Multimap<Geometry, Edge> geomBaseEdgeMap =
      HashMultimap.create();
  private final Multimap<Geometry, Edge> geomTurnEdgeMap =
      HashMultimap.create();

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

  private final DataCube dc;

  public OtpGraph(String path, String dcPath) {
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
    if (turnGraph == null) {
      throw new RuntimeException("Could not load graph (path=" + path
          + ")");
    }

    baseGraph = turnGraph.getService(BaseGraph.class).getBaseGraph();

    baseIndexService = new StreetVertexIndexServiceImpl(baseGraph);
    createIndices(baseGraph, baseEdgeIndex, null, geomBaseEdgeMap);
    createIndices(turnGraph, turnEdgeIndex, turnVertexIndex,
        geomTurnEdgeMap);

    if (dcPath == null)
      dc = new DataCube();
    else
      dc = DataCube.read(new File(dcPath));

    log.info("Graph loaded..");
  }

  private Set<InferredPath> computePaths(PathKey key) {

    /*
     * We always consider moving off of an edge, staying on an edge, and
     * whatever else we can find.
     */
    final VehicleState currentState = key.getState();
    final InferredEdge currentEdge = currentState.getBelief().getEdge().getInferredEdge();

    final Coordinate toCoord = key.getEndCoord();

    final Set<InferredPath> paths =
        Sets.newHashSet(InferredPath.getEmptyPath());
    final Set<Edge> startEdges = Sets.newHashSet();

    if (!currentEdge.isEmptyEdge()) {

      final PlainStreetEdgeWithOSMData edge =
          (PlainStreetEdgeWithOSMData) currentEdge.getEdge();
      /*
       * Make sure we get the non-base edges corresponding to our
       * current edge and the reverse.
       * XXX This violates all directionality and graph structure.
       */
      final Collection<Edge> turnEdges =
          geomTurnEdgeMap.get(edge.getTurnVertex().geometry);
      startEdges.addAll(turnEdges);
    } else {

      final MultivariateGaussian obsBelief =
          currentState.getMovementFilter().getObservationBelief(currentState.getBelief());

      final double beliefDistance =
          Math.min(StatisticsUtil
              .getLargeNormalCovRadius((DenseMatrix) obsBelief
                  .getCovariance()), MAX_STATE_SNAP_RADIUS);

      final Coordinate fromCoord = key.getStartCoord();
      for (final Object obj : getNearbyEdges(fromCoord,
          beliefDistance)) {
        final PlainStreetEdgeWithOSMData edge =
            (PlainStreetEdgeWithOSMData) obj;
        startEdges.addAll(edge.getTurnVertex().getOutgoing());
      }
    }

    final Set<Edge> endEdges = Sets.newHashSet();

    final double obsStdDevDistance =
        Math.min(StatisticsUtil
            .getLargeNormalCovRadius((DenseMatrix) currentState
                .getMovementFilter().getObsCovar()),
            MAX_OBS_SNAP_RADIUS);

    double maxEndEdgeLength = Double.NEGATIVE_INFINITY;
    for (final Object obj : getNearbyEdges(toCoord, obsStdDevDistance)) {

      final PlainStreetEdgeWithOSMData edge =
          (PlainStreetEdgeWithOSMData) obj;
      if (edge.getLength() > maxEndEdgeLength)
        maxEndEdgeLength = edge.getLength();

      final Collection<Edge> turnEdges =
          geomTurnEdgeMap.get(edge.getTurnVertex().geometry);
      endEdges.addAll(turnEdges);
    }

    if (endEdges.isEmpty())
      return paths;

    /*
     * If we're already on an edge, then we attempt to gauge how
     * far in the opposite direction we are willing to consider.
     */
    final double timeDiff =
        currentState.getMovementFilter().getCurrentTimeDiff();
    final double edgeLength =
        currentEdge.isEmptyEdge() ? 0d : currentEdge.getLength();
    final double distanceMax =
        Math.max(
            MAX_DISTANCE_SPEED * timeDiff,
            currentState.getMeanLocation().euclideanDistance(
                currentState.getObservation().getProjectedPoint()))
            + edgeLength + maxEndEdgeLength;

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
    if (gpath.edges.isEmpty()) {
      path.add(PathEdge.getEdge(this.getInferredEdge(startEdge), 0d,
          isReverse));
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
    STRtree vertexIndex, Multimap<Geometry, Edge> geomEdgeMap) {

    for (final Vertex v : graph.getVertices()) {
      if (vertexIndex != null) {
        final Envelope vertexEnvelope =
            new Envelope(v.getCoordinate());
        vertexIndex.insert(vertexEnvelope, v);
      }

      for (final Edge e : v.getOutgoing()) {
        final Geometry geometry = e.getGeometry();
        if (geometry != null) {
          if (geomEdgeMap != null) {
            geomEdgeMap.put(geometry, e);
            // TODO reverse shouldn't make a difference
            // if topological equality is used.  Is that
            // what's happening here?
            geomEdgeMap.put(geometry.reverse(), e);
          }

          if (graph.getIdForEdge(e) != null) {
            final Envelope envelope = geometry.getEnvelopeInternal();
            edgeIndex.insert(envelope, e);
          }
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

  public DataCube getDataCube() {
    return this.dc;
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
      if (((StreetEdge) obj).canTraverse(defaultOptions)
      //          && defaultOptions.getModes().contains(((StreetEdge) obj).getMode())
      )
        streetEdges.add((StreetEdge) obj);
    }
    return streetEdges;
  }

  public List<StreetEdge> getNearbyEdges(
    MultivariateGaussian initialBelief,
    AbstractRoadTrackingFilter trackingFilter) {
    Preconditions.checkArgument(initialBelief
        .getInputDimensionality() == 4);

    final Envelope toEnv =
        new Envelope(
            GeoUtils.makeCoordinate(AbstractRoadTrackingFilter
                .getOg().times(initialBelief.getMean())));
    final double varDistance =
        StatisticsUtil
            .getLargeNormalCovRadius((DenseMatrix) trackingFilter
                .getObsCovar());

    toEnv.expandBy(varDistance);

    final List<StreetEdge> streetEdges = Lists.newArrayList();
    for (final Object obj : baseEdgeIndex.query(toEnv)) {
      final Edge edge = (Edge) obj;
      if (((StreetEdge) edge).canTraverse(defaultOptions)
      //          && defaultOptions.getModes().contains(((StreetEdge) edge).getMode())
      )
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
    if (!fromState.getBelief().getEdge().getInferredEdge().isEmptyEdge()) {
      fromCoord = fromState.getBelief().getEdge().getInferredEdge().getCenterPointCoord();
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

  public Set<InferredEdge> getTopoEquivEdges(InferredEdge edge) {
    final Collection<Edge> baseEdges =
        geomBaseEdgeMap.get(edge.getGeometry());
    final Set<InferredEdge> results = Sets.newHashSet();
    for (final Edge bEdge : baseEdges) {
      results.add(this.getInferredEdge(bEdge));
    }
    return results;
  }

  public Graph getTurnGraph() {
    return turnGraph;
  }

  private PathEdge getValidPathEdge(Edge originalEdge,
    double pathDist, double direction, List<PathEdge> path) {
    final Edge edge = getBaseEdge(originalEdge);
    if (OtpGraph.isStreetEdge(edge) && edge.getGeometry() != null
        && edge.getDistance() > 0d
        && baseGraph.getIdForEdge(edge) != null
        && !edge.equals(Iterables.getLast(path, null))) {

      return PathEdge.getEdge(this.getInferredEdge(edge), pathDist,
          direction < 0d);

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
              pathDist, direction < 0d);
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

  public void writeDataCube(File outFile) {
    DataCube.write(this.dc, outFile);
  }

  public static List<Edge>
      filterForStreetEdges(Collection<Edge> edges) {
    final List<Edge> result = Lists.newArrayList();
    for (final Edge out : edges) {
      if ((out instanceof StreetEdge)
          && ((StreetEdge) out).canTraverse(defaultOptions)
      //          && defaultOptions.getModes().contains(out.getMode())
      ) {
        result.add(out);
      }
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
