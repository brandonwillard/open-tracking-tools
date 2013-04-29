package org.opentrackingtools.graph.otp;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DistributionWithMean;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.netlib.blas.BLAS;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentripplanner.routing.algorithm.GenericAStar;
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

public class OtpGraph implements InferenceGraph {

  public static class VertexPair {

    private final Vertex endVertex;
    private final Vertex startVertex;

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
      if (this.getClass() != obj.getClass()) {
        return false;
      }
      final VertexPair other = (VertexPair) obj;
      if (this.endVertex == null) {
        if (other.endVertex != null) {
          return false;
        }
      } else if (!this.endVertex.equals(other.endVertex)) {
        return false;
      }
      if (this.startVertex == null) {
        if (other.startVertex != null) {
          return false;
        }
      } else if (!this.startVertex.equals(other.startVertex)) {
        return false;
      }
      return true;
    }

    public Vertex getEndVertex() {
      return this.endVertex;
    }

    public Vertex getStartVertex() {
      return this.startVertex;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result =
          prime
              * result
              + ((this.endVertex == null) ? 0 : this.endVertex
                  .hashCode());
      result =
          prime
              * result
              + ((this.startVertex == null) ? 0 : this.startVertex
                  .hashCode());
      return result;
    }

  }

  private final static RoutingRequest defaultOptions =
      new RoutingRequest(new TraverseModeSet(TraverseMode.CAR));

  private static final Logger log = LoggerFactory
      .getLogger(OtpGraph.class);

  private static final double MAX_DISTANCE_SPEED = 53.6448; // ~120 mph

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

  public static List<Edge>
      filterForStreetEdges(Collection<Edge> edges) {
    final List<Edge> result = Lists.newArrayList();
    for (final Edge out : edges) {
      if ((out instanceof StreetEdge)
          && ((StreetEdge) out).canTraverse(OtpGraph.defaultOptions)
      //          && defaultOptions.getModes().contains(out.getMode())
      ) {
        result.add(out);
      }
    }
    return result;
  }

  public static RoutingRequest getDefaultoptions() {
    return OtpGraph.defaultOptions;
  }

  public static double getMaxDistanceSpeed() {
    return OtpGraph.MAX_DISTANCE_SPEED;
  }

  public static double getMaxObsSnapRadius() {
    return OtpGraph.MAX_OBS_SNAP_RADIUS;
  }

  public static double getMaxStateSnapRadius() {
    return OtpGraph.MAX_STATE_SNAP_RADIUS;
  }

  public static boolean isStreetEdge(Edge pathEdge) {
    if (!(pathEdge instanceof StreetEdge)) {
      return false;
    } else {
      return true;
    }
  }

  private final STRtree baseEdgeIndex = new STRtree();
  
  /**
   * This is the original (intersection-and-street-segment) graph, used for all
   * inference tasks other than routing.
   */
  private final Graph baseGraph;
  
  private final Map<VertexPair, InferenceGraphEdge> edgeToInfo = Maps
      .newConcurrentMap();

  private final Multimap<Geometry, Edge> geomBaseEdgeMap =
      HashMultimap.create();

  private final Multimap<Geometry, Edge> geomTurnEdgeMap =
      HashMultimap.create();

  private final GraphServiceImpl gs;

  //  private Set<Path> computePaths(PathKey key) {
  //
  //    /*
  //     * We always consider moving off of an edge, staying on an edge, and
  //     * whatever else we can find.
  //     */
  //    final VehicleState currentState = key.getState();
  //    final InferenceGraphEdge currentEdge =
  //        currentState.getBelief().getEdge()
  //            .getInferenceGraphEdge();
  //    
  //    Preconditions.checkArgument(
  //        currentEdge.isNullEdge()
  //        || (currentEdge.getBackingEdge() instanceof PlainStreetEdgeWithOSMData));
  //
  //    final Coordinate toCoord = key.getEndCoord();
  //
  //    final Set<Path> paths =
  //        Sets.newHashSet(SimplePath.getNullPath());
  //    final Set<Edge> startEdges = Sets.newHashSet();
  //
  //    if (!currentEdge.isNullEdge()) {
  //
  //      final PlainStreetEdgeWithOSMData edge =
  //          (PlainStreetEdgeWithOSMData) currentEdge
  //              .getBackingEdge();
  //      /*
  //       * Make sure we get the non-base edges corresponding to our
  //       * current edge and the reverse.
  //       */
  //      final Collection<Edge> turnEdges =
  //          geomTurnEdgeMap
  //              .get(edge.getTurnVertex().geometry);
  //      // XXX This violates all directionality and graph structure.
  //      startEdges.addAll(turnEdges);
  //    } else {
  //
  //      final MultivariateGaussian obsBelief =
  //          currentState.getMovementFilter()
  //              .getObservationBelief(
  //                  currentState.getBelief());
  //
  //      final double beliefDistance =
  //          Math.min(
  //              StatisticsUtil
  //                  .getLargeNormalCovRadius((DenseMatrix) obsBelief
  //                      .getCovariance()),
  //              MAX_STATE_SNAP_RADIUS);
  //
  //      final Coordinate fromCoord = key.getStartCoord();
  //      for (final Object obj : getNearbyOtpEdges(fromCoord,
  //          beliefDistance)) {
  //        final PlainStreetEdgeWithOSMData edge =
  //            (PlainStreetEdgeWithOSMData) obj;
  //        startEdges.addAll(edge.getTurnVertex()
  //            .getOutgoing());
  //      }
  //    }
  //
  //    final Set<Edge> endEdges = Sets.newHashSet();
  //
  //    final double obsStdDevDistance =
  //        Math.min(
  //            StatisticsUtil
  //                .getLargeNormalCovRadius((DenseMatrix) currentState
  //                    .getMovementFilter().getObsCovar()),
  //            MAX_OBS_SNAP_RADIUS);
  //
  //    double maxEndEdgeLength = Double.NEGATIVE_INFINITY;
  //    for (final StreetEdge obj: getNearbyOtpEdges(toCoord,
  //        obsStdDevDistance)) {
  //
  //      final PlainStreetEdgeWithOSMData edge =
  //          (PlainStreetEdgeWithOSMData) obj;
  //      if (edge.getLength() > maxEndEdgeLength)
  //        maxEndEdgeLength = edge.getLength();
  //
  //      final Collection<Edge> turnEdges =
  //          geomTurnEdgeMap
  //              .get(edge.getTurnVertex().geometry);
  //      endEdges.addAll(turnEdges);
  //    }
  //
  //    if (endEdges.isEmpty())
  //      return paths;
  //
  //    /*
  //     * If we're already on an edge, then we attempt to gauge how
  //     * far in the opposite direction we are willing to consider.
  //     */
  //    final double timeDiff =
  //        currentState.getMovementFilter()
  //            .getCurrentTimeDiff();
  //    final double edgeLength =
  //        currentEdge.isNullEdge() ? 0d : currentEdge
  //            .getLength();
  //    final double distanceMax =
  //        Math.max(
  //            MAX_DISTANCE_SPEED * timeDiff,
  //            currentState.getMeanLocation()
  //                .euclideanDistance(
  //                    currentState.getObservation()
  //                        .getProjectedPoint()))
  //            + edgeLength + maxEndEdgeLength;
  //
  //    for (final Edge startEdge : startEdges) {
  //      final MultiDestinationAStar forwardAStar =
  //          new MultiDestinationAStar(turnGraph, endEdges,
  //              toCoord, obsStdDevDistance, startEdge,
  //              distanceMax);
  //
  //      final ShortestPathTree spt1 =
  //          forwardAStar.getSPT(false);
  //
  ////      final MultiDestinationAStar backwardAStar =
  ////          new MultiDestinationAStar(turnGraph, endEdges,
  ////              toCoord, obsStdDevDistance, startEdge,
  ////              distanceMax);
  ////      final ShortestPathTree spt2 =
  ////          backwardAStar.getSPT(true);
  //
  //      for (final Edge endEdge : endEdges) {
  //        final GraphPath forwardPath =
  //            spt1.getPath(endEdge.getToVertex(), false);
  //        if (forwardPath != null) {
  //          /*
  //           * Just to be safe we check the end location's
  //           * distance to the obsevation.
  //           */
  //          final Path forwardResult =
  //              copyAStarResults(forwardPath,
  //                  getBaseEdge(startEdge), false);
  //          
  //          if (forwardResult != null) {
  //            final double distToObs = 
  //                Iterables.getLast(forwardResult.getPathEdges()).getGeometry().distance(
  //                    JTSFactoryFinder.getGeometryFactory().createPoint(
  //                        toCoord));
  //            if (distToObs - obsStdDevDistance > 0) {
  //              log.debug("path search did not stop within allowable distance");
  //            } else {
  //              paths.add(forwardResult);
  //            }
  //          }
  //        }
  //
  ////        if (backwardAStar != null) {
  ////          final GraphPath backwardPath =
  ////              spt2.getPath(endEdge.getFromVertex(), false);
  ////          if (backwardPath != null) {
  ////            final Path backwardResult =
  ////                copyAStarResults(backwardPath, startEdge,
  ////                    true);
  ////            if (backwardResult != null) {
  ////            final double distToObs = 
  ////                Iterables.getLast(backwardResult.getPathEdges()).getGeometry().distance(
  ////                    JTSFactoryFinder.getGeometryFactory().createPoint(
  ////                        toCoord));
  ////              if (distToObs - obsStdDevDistance > 0) {
  ////                log.info("path search did not stop within allowable distance");
  ////              } else {
  ////                paths.add(backwardResult);
  ////              }
  ////            }
  ////          }
  ////        }
  //      }
  //    }
  //
  //    return paths;
  //  }

  //  private Set<Path> computeUniquePaths(PathKey key) {
  //    final Set<Path> paths = computePaths(key);
  //    makeUnique(paths);
  //    return paths;
  //  }
  //
  //  private Path copyAStarResults(GraphPath gpath,
  //    Edge startEdge, boolean isReverse) {
  //    final double direction = isReverse ? -1d : 1d;
  //    double pathDist = 0d;
  //    final List<PathEdge> path = Lists.newArrayList();
  //    if (gpath.edges.isEmpty()) {
  //      path.add(SimplePathEdge.getEdge(
  //          this.getInferenceGraphEdge(startEdge), 0d, isReverse));
  //    } else {
  //      for (final Edge edge : isReverse ? Lists
  //          .reverse(gpath.edges) : gpath.edges) {
  //        final PathEdge pathEdge =
  //            getValidPathEdge(edge, pathDist, direction,
  //                path);
  //        pathDist += direction * pathEdge.getLength();
  //        path.add(pathEdge);
  //      }
  //    }
  //    if (!path.isEmpty())
  //      return OtpPath.getPath(path, isReverse);
  //    else
  //      return null;
  //  }

  private final STRtree turnEdgeIndex = new STRtree();

  /**
   * This is the edge-based graph used in routing; neither it nor any edges form
   * it should ever be used outside of this class.
   */
  private final Graph turnGraph;

  private final Envelope turnGraphExtent;

  //base index service is in projected coords
  private final StreetVertexIndexServiceImpl turnIndexService;

  private final STRtree turnVertexIndex = new STRtree();

  public OtpGraph(String path) {
    OtpGraph.log.info("Loading OTP graph...");
    OtpGraph.log.info("Using BLAS: "
        + BLAS.getInstance().getClass().getName());
    this.gs = new GraphServiceImpl();
    this.gs.setLoadLevel(LoadLevel.DEBUG);

    final ApplicationContext appContext =
        new GenericApplicationContext();

    this.gs.setResourceLoader(appContext);

    this.gs.setPath(path);
    this.gs.refreshGraphs();

    this.turnGraph = this.gs.getGraph();
    if (this.turnGraph == null) {
      throw new RuntimeException("Could not load graph (path=" + path
          + ")");
    }

    this.baseGraph =
        this.turnGraph.getService(BaseGraph.class).getBaseGraph();

    // FIXME do this now, since adding temp vertices makes
    // getVertices freak out with ConcurrentModificationExceptions
    this.turnGraphExtent = this.turnGraph.getExtent();

    this.turnIndexService =
        new StreetVertexIndexServiceImpl(this.turnGraph);
    this.createIndices(this.baseGraph, this.baseEdgeIndex, null,
        this.geomBaseEdgeMap);
    this.createIndices(this.turnGraph, this.turnEdgeIndex,
        this.turnVertexIndex, this.geomTurnEdgeMap);

    OtpGraph.log.info("Graph loaded..");
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

  @Override
  public boolean edgeHasReverse(Geometry edge) {
    final Collection<Edge> baseEdges =
        this.getGeomBaseEdgeMap().get(edge);
    boolean hasReverseTmp = false;
    for (final Edge bEdge : baseEdges) {
      if (bEdge.getGeometry().reverse().equalsExact(edge)) {
        hasReverseTmp = true;
      }
    }
    return hasReverseTmp;
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
    return this.baseGraph;
  }

  public Map<VertexPair, InferenceGraphEdge> getEdgeToInfo() {
    return this.edgeToInfo;
  }

  public Multimap<Geometry, Edge> getGeomBaseEdgeMap() {
    return this.geomBaseEdgeMap;
  }

  public Multimap<Geometry, Edge> getGeomTurnEdgeMap() {
    return this.geomTurnEdgeMap;
  }

  @Override
  public Envelope getGPSGraphExtent() {
    return this.turnGraphExtent;
  }

  public GraphServiceImpl getGs() {
    return this.gs;
  }

  /**
   * This returns a list of edges that are incoming, wrt the direction of this
   * edge, and that are reachable from this edge (e.g. not one way in the
   * direction of this edge).
   * 
   * @return
   */
  @Override
  public List<InferenceGraphEdge> getIncomingTransferableEdges(
    InferenceGraphEdge infEdge) {

    final List<InferenceGraphEdge> result = Lists.newArrayList();
    for (final Edge edge : OtpGraph
        .filterForStreetEdges(((Edge) (infEdge.getBackingEdge()))
            .getFromVertex().getIncoming())) {
      if (this.getBaseGraph().getIdForEdge(edge) != null) {
        result.add(this.getInferenceGraphEdge(edge));
      }
    }

    return result;
  }

  public InferenceGraphEdge getInferenceGraphEdge(Edge edge) {
    edge = this.getBaseEdge(edge);

    final VertexPair key =
        new VertexPair(edge.getFromVertex(), edge.getToVertex());
    InferenceGraphEdge edgeInfo = this.edgeToInfo.get(key);

    if (edgeInfo == null) {
      final Integer edgeId = this.baseGraph.getIdForEdge(edge);
      edgeInfo =
          new InferenceGraphEdge(edge.getGeometry(), edge, edgeId,
              this);
      this.edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  //  private PathEdge getValidPathEdge(Edge originalEdge,
  //    double pathDist, double direction, List<PathEdge> path) {
  //    final Edge edge = getBaseEdge(originalEdge);
  //    if (OtpGraph.isStreetEdge(edge)
  //        && edge.getGeometry() != null
  //        && edge.getDistance() > 0d
  //        && baseGraph.getIdForEdge(edge) != null
  //        && !edge.equals(Iterables.getLast(path, null))) {
  //
  //      return new PathEdge(this.getInferenceGraphEdge(edge),
  //          pathDist, direction < 0d);
  //
  //    } else if (edge.getFromVertex() != null
  //        && !edge.getFromVertex().getOutgoingStreetEdges()
  //            .isEmpty()) {
  //
  //      for (final Edge streetEdge : edge.getFromVertex()
  //          .getOutgoingStreetEdges()) {
  //
  //        if (streetEdge.getGeometry() != null
  //            && !streetEdge.equals(Iterables.getLast(path,
  //                null)) && streetEdge.getDistance() > 0d
  //            && baseGraph.getIdForEdge(streetEdge) != null) {
  //
  //          /*
  //           * Find a valid street edge to work with
  //           */
  //          return new PathEdge(
  //              this.getInferenceGraphEdge(streetEdge), pathDist,
  //              direction < 0d);
  //        }
  //      }
  //    }
  //
  //    return null;
  //  }

  @Override
  public InferenceGraphEdge getInferenceGraphEdge(String strId) {

    final int id = Integer.parseInt(strId);
    final Edge edge = this.baseGraph.getEdgeById(id);
    final VertexPair key =
        new VertexPair(edge.getFromVertex(), edge.getToVertex());
    InferenceGraphEdge edgeInfo = this.edgeToInfo.get(key);

    if (edgeInfo == null) {
      edgeInfo =
          new InferenceGraphEdge(edge.getGeometry(), edge, id, this);
      this.edgeToInfo.put(key, edgeInfo);
    }

    return edgeInfo;
  }

  public Collection<InferenceGraphEdge> getInferenceGraphEdges() {
    return this.edgeToInfo.values();
  }

  @Override
  public Collection<InferenceGraphSegment> getNearbyEdges(
    Coordinate toCoord, double radius) {

    final Envelope toEnv = new Envelope(toCoord);
    toEnv.expandBy(radius);
    final Set<InferenceGraphSegment> streetEdges = Sets.newHashSet();
    for (final Object obj : this.baseEdgeIndex.query(toEnv)) {
      final StreetEdge edge = (StreetEdge) obj;
      if (edge.canTraverse(OtpGraph.defaultOptions)) {
        
        /*
         * FIXME we don't want to do this scanning, yet we don't
         * want to instantiate every inference graph edge.
         */
        final InferenceGraphEdge infEdge = getInferenceGraphEdge(edge);
        for (InferenceGraphSegment segment : infEdge.getSegments()) {
//          Preconditions.checkState(segment.getEndIndex()
//              .getSegmentIndex()
//              - segment.getStartIndex().getSegmentIndex() == 1);
          if (segment.getLine().distance(toCoord) < radius) {
            streetEdges.add(segment);
          } 
        }
      }
    }
    return streetEdges;
  }

  @Override
  public Collection<InferenceGraphSegment> getNearbyEdges(
    DistributionWithMean<Vector> initialBelief, Matrix covar) {

    Preconditions.checkArgument(initialBelief.getMean()
        .getDimensionality() == 4);

    final Vector toLoc =
        MotionStateEstimatorPredictor.getOg().times(
            initialBelief.getMean());
    final double varDistance =
        StatisticsUtil.getLargeNormalCovRadius(covar);

    return this.getNearbyEdges(toLoc, varDistance);
  }

  //  /**
  //   * Assume that paths are a subset of a shortest path tree. Remove all paths
  //   * that are either duplicates or are subpaths of a longer path.
  //   * 
  //   * @param paths
  //   */
  //  public static void makeUnique(Set<? extends Path> paths) {
  //    final PathTree tree = new PathTree();
  //
  //    final HashSet<Path> toRemove =
  //        new HashSet<Path>();
  //    for (final Path path : paths) {
  //      PathTree cur = tree;
  //      for (final PathEdge edge : path.getPathEdges()) {
  //        cur = cur.apply(edge, path);
  //        if (cur.isLeaf()) {
  //          //we are visiting a node that was previously a leaf.  It is no longer a leaf
  //          //and the paths that had previously visited it should be removed.
  //          cur.isLeaf = false;
  //          toRemove.addAll(cur.paths);
  //          assert (cur.paths.size() == 1);
  //          cur.removePath(cur.paths.get(0));
  //        }
  //        cur.paths.add(path);
  //      }
  //      if (!cur.isLeaf) {
  //        //either this is a true internal node
  //        //or this is the first time we have visited it
  //        if (cur.children.size() == 0) {
  //          cur.isLeaf = true;
  //        } else {
  //          toRemove.add(path);
  //          cur.removePath(path);
  //        }
  //      }
  //    }
  //    paths.removeAll(toRemove);
  //  }

  @Override
  public Collection<InferenceGraphSegment> getNearbyEdges(
    Vector projLocation, double radius) {
    Preconditions
        .checkArgument(projLocation.getDimensionality() == 2);
    return this.getNearbyEdges(GeoUtils.makeCoordinate(projLocation),
        radius);
  }

  public RoutingRequest getOptions() {
    return OtpGraph.defaultOptions;
  }

  /**
   * This returns a list of edges that are outgoing, wrt the direction of this
   * edge, and that are reachable from this edge (e.g. not one way against the
   * direction of this edge).
   * 
   * @return
   */
  @Override
  public List<InferenceGraphEdge> getOutgoingTransferableEdges(
    InferenceGraphEdge infEdge) {

    final List<InferenceGraphEdge> result = Lists.newArrayList();
    for (final Edge edge : OtpGraph
        .filterForStreetEdges(((Edge) (infEdge.getBackingEdge()))
            .getToVertex().getOutgoingStreetEdges())) {
      result.add(this.getInferenceGraphEdge(edge));
    }

    return result;
  }

  public List<Integer> getPathBetweenPoints(Coordinate fromCoord,
    Coordinate toCoord) {

    final RoutingRequest options = OtpGraph.defaultOptions;
    final CandidateEdgeBundle fromEdges =
        this.turnIndexService.getClosestEdges(new Coordinate(
            fromCoord.y, fromCoord.x), options, null, null);
    final CandidateEdgeBundle toEdges =
        this.turnIndexService.getClosestEdges(new Coordinate(
            toCoord.y, fromCoord.x), options, null, null);

    final GenericAStar astar = new GenericAStar();
    final RoutingRequest req = new RoutingRequest(options.getModes());
    /*
     * TODO which vertices to use?
     */
    final Vertex fromVertex =
        Iterables.getFirst(fromEdges.toEdgeList(), null)
            .getFromVertex(); //fromEdges.best.edge.getFromVertex();
    final Vertex toVertex =
        Iterables.getFirst(toEdges.toEdgeList(), null).getToVertex(); //toEdges.best.edge.getFromVertex();
    req.setRoutingContext(this.turnGraph, fromVertex, toVertex);
    final ShortestPathTree tree = astar.getShortestPathTree(req);

    final GraphPath result =
        Iterables.getFirst(tree.getPaths(), null);
    final List<Integer> edgeIds = Lists.newArrayList();

    for (final Edge edge : result.edges) {
      final Edge bEdge = this.getBaseEdge(edge);
      final Integer id = this.baseGraph.getIdForEdge(bEdge);
      if (id != null) {
        edgeIds.add(id);
      }
    }

    return edgeIds;
  }

  @Override
  public Set<Path> getPaths(VehicleStateDistribution fromState,
    GpsObservation obs) {
    return null;
  }

  @Override
  public Envelope getProjGraphExtent() {
    return this.getBaseGraph().getExtent();
  }

  @Override
  public Set<InferenceGraphEdge> getTopoEquivEdges(
    InferenceGraphEdge edge) {
    final Collection<Edge> baseEdges =
        this.geomBaseEdgeMap.get(edge.getGeometry());
    final Set<InferenceGraphEdge> results = Sets.newHashSet();
    for (final Edge bEdge : baseEdges) {
      results.add(this.getInferenceGraphEdge(bEdge));
    }
    return results;
  }

  public Graph getTurnGraph() {
    return this.turnGraph;
  }

  public StreetVertexIndexServiceImpl getTurnIndexService() {
    return this.turnIndexService;
  }

  public int getVertexCount() {
    return this.baseGraph.getVertices().size();
  }

}
