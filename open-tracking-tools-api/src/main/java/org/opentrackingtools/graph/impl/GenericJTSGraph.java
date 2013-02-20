package org.opentrackingtools.graph.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.DistributionWithMean;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.graph.build.line.DirectedLineStringGraphGenerator;
import org.geotools.graph.path.AStarShortestPathFinder;
import org.geotools.graph.path.Path;
import org.geotools.graph.structure.DirectedEdge;
import org.geotools.graph.structure.Edge;
import org.geotools.graph.structure.Node;
import org.geotools.graph.structure.basic.BasicDirectedEdge;
import org.geotools.graph.structure.basic.BasicDirectedNode;
import org.geotools.graph.traverse.standard.AStarIterator.AStarFunctions;
import org.geotools.graph.traverse.standard.AStarIterator.AStarNode;
import org.geotools.graph.util.geom.GeometryUtil;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.TrueObservation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateFilter;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.CoordinateSequenceFilter;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryComponentFilter;
import com.vividsolutions.jts.geom.GeometryFilter;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.index.strtree.STRtree;

public class GenericJTSGraph implements InferenceGraph {

  /**
   * Assuming that the LineString is mostly constant allows
   * us to cache values like getLength, which otherwise, over time,
   * build up needless calculations.
   * If the internal values happen to change, then we update
   * the cached values anyway.
   * @author bwillard
   *
   */
  public class ConstLineString extends LineString {

    private static final long serialVersionUID = 1114083576711858849L;
    
    double length;
    
    public ConstLineString(LineString projectedEdge) {
      super(projectedEdge.getCoordinateSequence(), projectedEdge.getFactory());
      length = projectedEdge.getLength();
    }

    @Override
    public double getLength() {
      return length;
    }

    @Override
    protected void geometryChangedAction() {
      super.geometryChangedAction();
      length = super.getLength();
    }
  }

  public class StrictLineStringGraphGenerator extends
      DirectedLineStringGraphGenerator {

    public StrictLineStringGraphGenerator() {
      super();
      setGraphBuilder(new StrictDirectedGraphBuilder());
    }
  }

  protected DirectedLineStringGraphGenerator graphGenerator = null;
  
  protected STRtree edgeIndex = null;
  
  protected static final double MAX_DISTANCE_SPEED = 53.6448; // ~120 mph
  
  protected static final Logger log = LoggerFactory
      .getLogger(GenericJTSGraph.class);

  /*
   * Maximum radius we're willing to search around a given
   * observation when snapping (for path search destination edges)
   */
  protected static final double MAX_OBS_SNAP_RADIUS = 200d;

  /*
   * Maximum radius we're willing to search around a given
   * state when snapping (for path search off -> on-road edges)
   */
  protected static final double MAX_STATE_SNAP_RADIUS = 350d;

  protected Envelope gpsEnv = null;
  protected Envelope projEnv = null;
  
  protected GenericJTSGraph() {
  }
  
  /**
   * The given collection of lines should be in GPS coordinates,
   * since they will be projected here.
   * @param lines
   */
  public GenericJTSGraph(Collection<LineString> lines) {
    createGraphFromLineStrings(lines);
  }
  
  protected void createGraphFromLineStrings(Collection<LineString> lines) {
    graphGenerator = new StrictLineStringGraphGenerator(); 
    edgeIndex = new STRtree();
    gpsEnv = new Envelope();
    projEnv = new Envelope();
    for (LineString edge : lines) {
      gpsEnv.expandToInclude(edge.getEnvelopeInternal());
      final MathTransform transform = GeoUtils.getTransform(edge.getCoordinate());
      Geometry projectedEdge; 
      try {
        projectedEdge = JTS.transform(edge, transform);
        projEnv.expandToInclude(projectedEdge.getEnvelopeInternal());
        final ConstLineString constLine = new ConstLineString((LineString)projectedEdge);
        constLine.setUserData(edge);
        graphGenerator.add(constLine);
      } catch (final TransformException e) {
        e.printStackTrace();
      }
    }
    /*
     * Initialize the id map and edge index.
     * 
     * The edge index is build from the line segments of
     * the geoms, so that distance calculations won't 
     * slow things down when querying for nearby edges.
     * 
     * TODO is there some way to do this lazily?  the
     * general problem is that we might want to query
     * an edge by it's id, yet it hasn't been initialized,
     * so it doesn't get into the map (by the way, we
     * have to keep our own map; the internal graph doesn't
     * do that).
     */
    for (Object obj : graphGenerator.getGraph().getEdges()) {
      final BasicDirectedEdge edge = (BasicDirectedEdge) obj;
      InferredEdge infEdge = getInferredEdge(edge);
      final LineString lineString = (LineString) infEdge.getGeometry();
      for (LineSegment line : GeoUtils.getSubLineSegments(lineString)) {
        Pair<LineSegment, InferredEdge> lineEdgePair = DefaultPair.create(line, infEdge);
        edgeIndex.insert(new Envelope(line.p0, line.p1), lineEdgePair);
      }
      
    }
    edgeIndex.build();
  }
  
  public static class VehicleStateAStarFunction extends AStarFunctions {
    
    final Coordinate toCoord;
    final double obsStdDevDistance;
    
    public VehicleStateAStarFunction(Node destination, 
      Coordinate toCoord, double obsStdDevDistance) {
      super(destination);
      this.toCoord = toCoord;
      this.obsStdDevDistance = obsStdDevDistance;
    }

    @Override
    public double h(Node n) {
  
      final double distance =
          ((Point)n.getObject()).getCoordinate().distance(
              toCoord);
  
      if (distance < obsStdDevDistance)
        return 0d;
  
      return (distance - obsStdDevDistance)
          / 15d; // 15 m/s, ~35 mph, a random driving speed
    }
    
    @Override
    public double cost(AStarNode n1, AStarNode n2) {
      final BasicDirectedNode dn1 = (BasicDirectedNode) n1.getNode();
      final BasicDirectedNode dn2 = (BasicDirectedNode) n2.getNode();
      
      Edge edgeBetween = dn1.getOutEdge(dn2);
      /*
       * Make sure this direction is traversable
       */
      if (edgeBetween == null) {
        return Double.POSITIVE_INFINITY;
      }
      
      /*
       * If these are the first nodes, then make sure
       * they travel the direction of the edge
       */
//      if (n1.getParent() == null) {
//        if (!edgeBetween.equals(startEdge))
//          return Double.POSITIVE_INFINITY;
//      }
      
      /*
       * TODO
       * Compute distance past projected value
       */
      
      return 0d;
    }
  }
  
  @Override
  public Set<InferredPath> getPaths(final VehicleState fromState,
    final GpsObservation obs) {
    
    final Coordinate toCoord = obs.getObsProjected();
    
    InferredEdge currentEdge = fromState.getBelief().getEdge().getInferredEdge();
    
    Set<InferredEdge> startEdges = Sets.newHashSet();
    
    if (!currentEdge.isNullEdge()) {
      startEdges.add(currentEdge);
    } else {

      final MultivariateGaussian obsBelief =
          fromState.getMovementFilter()
              .getObservationBelief(
                  fromState.getBelief());

      final double beliefDistance =
          Math.min(
              StatisticsUtil
                  .getLargeNormalCovRadius((DenseMatrix) obsBelief
                      .getCovariance()),
              MAX_STATE_SNAP_RADIUS);

      startEdges.addAll(getNearbyEdges(
            fromState.getMeanLocation(), beliefDistance));
    }


    final double obsStdDevDistance =
        Math.min(
            StatisticsUtil
                .getLargeNormalCovRadius((DenseMatrix) fromState
                    .getMovementFilter().getObsCovar()),
            MAX_OBS_SNAP_RADIUS);

    final Set<Node> endNodes= Sets.newHashSet();
    for (final InferredEdge edge: getNearbyEdges(toCoord,
        obsStdDevDistance)) {
      final DirectedEdge bEdge = ((DirectedEdge)edge.getBackingEdge());
      endNodes.add(bEdge.getNodeA());
      endNodes.add(bEdge.getNodeB());
    }

    Set<InferredPath> paths = Sets.newHashSet();
    paths.add(getNullPath());
    
    if (endNodes.isEmpty())
      return paths;
    
    
    for (InferredEdge startEdge : startEdges) {
      
      // TODO FIXME determine when/how backward movement fits in
      paths.add(getInferredPath(getPathEdge(startEdge, 0, false)));
      
      final DirectedEdge bStartEdge = ((DirectedEdge)startEdge.getBackingEdge());
      final Node source = bStartEdge.getOutNode();
        
      /*
       * Use this set to avoid recomputing subpaths of 
       * our current paths.
       */
      Set<Node> reachedEndNodes = Sets.newHashSet(source);
      for (Node target : endNodes) {
        
        if (reachedEndNodes.contains(target))
          continue;
        
        AStarFunctions afuncs = new VehicleStateAStarFunction(target, 
            toCoord, obsStdDevDistance);
        
        CustomAStarShortestPathFinder aStarIter = new CustomAStarShortestPathFinder(
            this.graphGenerator.getGraph(), source, target, afuncs);
        aStarIter.calculate();
        
        Path path = aStarIter.getPath();
        
        if (path != null) {
          List<PathEdge> pathEdges = Lists.newArrayList();
          double distToStart = 0d;
          Iterator<?> iter = path.riterator();
          BasicDirectedNode prevNode = (BasicDirectedNode) bStartEdge.getInNode();
          while (iter.hasNext()) {
            BasicDirectedNode node = (BasicDirectedNode)iter.next();
            Edge edge = Preconditions.checkNotNull(prevNode.getOutEdge(node));
            
            InferredEdge infEdge = getInferredEdge(edge);
            pathEdges.add(getPathEdge(infEdge, distToStart, false));
            distToStart += infEdge.getLength();
            
            reachedEndNodes.add(node);
            prevNode = node;
          }
          if (!pathEdges.isEmpty())
            paths.add(getInferredPath(pathEdges, false));
        }
        
        // TODO backward paths? 
          
      }
    }
    
    // TODO debug; remove.
    if (obs instanceof TrueObservation) {
      final VehicleState trueState = ((TrueObservation)obs).getTrueState();
      if (!trueState.getBelief().getPath().isNullPath() && 
          fromState.getBelief().getEdge().getInferredEdge()
            .equals(
                Iterables.getFirst(trueState.getBelief().getPath().getPathEdges(), null).
                getInferredEdge())
            && Iterables.find(paths, 
                new Predicate<InferredPath>() {

                  @Override
                  public boolean apply(InferredPath input) {
                    return input.getGeometry() != null &&
                        input.getGeometry().covers(
                          trueState.getBelief().getPath().getGeometry());
                  }
                }
                , null) == null) {
        log.warn("True path not found in search results: true=" 
                + trueState.getBelief().getPath() + ", found=" + paths);
      }
    }

    return paths;
  }

  @Override
  public Set<InferredEdge> getTopoEquivEdges(InferredEdge edge) {
    Set<InferredEdge> result = Sets.newHashSet();
    /*
     * Get reverse edge, if it's there
     */
    Coordinate[] coords = edge.getGeometry().getCoordinates();
    Edge revEdge = graphGenerator.getEdge(coords[coords.length - 1], coords[0]);
    
    InferredEdge revInfEdge = getInferredEdge(revEdge);
    result.add(edge);
    result.add(revInfEdge);
    
    return result;
  }

  Map<Edge, InferredEdge> edgeToInfEdge = Maps.newHashMap();
  Map<String, InferredEdge> idToInfEdge = Maps.newHashMap();
  
  private InferredEdge getInferredEdge(Edge edge) {
    InferredEdge infEdge = edgeToInfEdge.get(edge);

    if (infEdge == null) {
      final Geometry edgeGeom = Preconditions.checkNotNull((Geometry)edge.getObject());
      final int id = edge.getID();
      infEdge = SimpleInferredEdge.getInferredEdge(edgeGeom, edge, 
          id, this);
      
      edgeToInfEdge.put(edge, infEdge);
      idToInfEdge.put(infEdge.getEdgeId(), infEdge);
    }

    return infEdge;
  }

  @Override
  public Envelope getGPSGraphExtent() {
    return gpsEnv;
  }

  @Override
  public Collection<InferredEdge> getNearbyEdges(
    DistributionWithMean<Vector> initialBelief,
    AbstractRoadTrackingFilter trackingFilter) {
    
    Preconditions.checkArgument(initialBelief.getMean()
        .getDimensionality() == 4);

    final Vector toLoc =
            AbstractRoadTrackingFilter
                .getOg().times(initialBelief.getMean());
    final double varDistance =
        StatisticsUtil
            .getLargeNormalCovRadius((DenseMatrix) trackingFilter
                .getObsCovar());

    return getNearbyEdges(toLoc, varDistance);
  }

  @Override
  public Collection<InferredEdge> getNearbyEdges(Vector projLocation,
    double radius) {
    Preconditions.checkArgument(projLocation.getDimensionality() == 2);
    return getNearbyEdges(GeoUtils.makeCoordinate(projLocation), radius);
  }
  
  public Collection<InferredEdge> getNearbyEdges(Coordinate toCoord,
    double radius) {
    final Envelope toEnv = new Envelope(toCoord);
    toEnv.expandBy(radius);
    final Set<InferredEdge> streetEdges = Sets.newHashSet();
    for (final Object obj : edgeIndex.query(toEnv)) {
      final Pair<LineSegment, InferredEdge> lineEdgePair = (Pair<LineSegment, InferredEdge>) obj;
      if (lineEdgePair.getFirst().distance(toCoord) < radius)
        streetEdges.add(lineEdgePair.getSecond());
      else
        continue;
    }
    return streetEdges;
  }

  @Override
  public InferredEdge getNullInferredEdge() {
    return SimpleInferredEdge.getNullEdge();
  }

  @Override
  public InferredPath getNullPath() {
    return SimpleInferredPath.getNullPath();
  }

  @Override
  public PathEdge getNullPathEdge() {
    return SimplePathEdge.getNullPathEdge();
  }

  @Override
  public Collection<InferredEdge> getIncomingTransferableEdges(
    InferredEdge infEdge) {
    
    DirectedEdge edge = (DirectedEdge) infEdge.getBackingEdge();
    Collection<DirectedEdge> inEdges = edge.getInNode().getInEdges();
    
    Set<InferredEdge> result = Sets.newHashSet();
    for (DirectedEdge inEdge : inEdges) {
      result.add(getInferredEdge(inEdge));
    }
                                         
    return result;
  }

  @Override
  public Collection<InferredEdge> getOutgoingTransferableEdges(
    InferredEdge infEdge) {
    
    DirectedEdge edge = (DirectedEdge) infEdge.getBackingEdge();
    Collection<DirectedEdge> outEdges = edge.getOutNode().getOutEdges();
    
    Set<InferredEdge> result = Sets.newHashSet();
    for (DirectedEdge outEdge : outEdges) {
      result.add(getInferredEdge(outEdge));
    }
                                         
    return result;
  }

  @Override
  public Envelope getProjGraphExtent() {
    return projEnv; 
  }

  @Override
  public PathEdge getPathEdge(InferredEdge edge, double d, Boolean b) {
    return SimplePathEdge.getEdge(edge, d, b);
  }

  @Override
  public InferredPath getInferredPath(PathEdge pathEdge) {
    return SimpleInferredPath.getInferredPath(pathEdge);
  }

  @Override
  public InferredPath getInferredPath(List<PathEdge> currentPath,
    Boolean b) {
    return SimpleInferredPath.getInferredPath(currentPath, b);
  }

  @Override
  public boolean edgeHasReverse(Geometry edge) {
    return this.graphGenerator.get(edge.reverse()) != null;
  }

  @Override
  public InferredEdge getInferredEdge(String id) {
    return idToInfEdge.get(id);
  }

  @Override
  public VehicleState createVehicleState(GpsObservation obs,
      AbstractRoadTrackingFilter trackingFilter,
      PathStateBelief pathStateBelief, OnOffEdgeTransDirMulti edgeTransDist,
      VehicleState parent) {
    return new VehicleState(this, obs, trackingFilter, 
        pathStateBelief, edgeTransDist, parent);
  }

}
