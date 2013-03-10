package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Matrix;
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
import org.geotools.graph.build.line.DirectedLineStringGraphGenerator;
import org.geotools.graph.structure.DirectedEdge;
import org.geotools.graph.structure.Edge;
import org.geotools.graph.structure.Node;
import org.geotools.graph.structure.basic.BasicDirectedEdge;
import org.geotools.graph.structure.basic.BasicDirectedNode;
import org.geotools.graph.traverse.standard.AStarIterator.AStarFunctions;
import org.geotools.graph.traverse.standard.AStarIterator.AStarNode;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.index.strtree.STRtree;

public class GenericJTSGraph implements InferenceGraph {

  /**
   * Assuming that the LineString is mostly constant allows us to cache values
   * like getLength, which otherwise, over time, build up needless calculations.
   * If the internal values happen to change, then we update the cached values
   * anyway.
   * 
   * @author bwillard
   * 
   */
  public class ConstLineString extends LineString {

    private static final long serialVersionUID = 1114083576711858849L;

    double length;

    public ConstLineString(LineString projectedEdge) {
      super(projectedEdge.getCoordinateSequence(), projectedEdge
          .getFactory());
      this.length = projectedEdge.getLength();
    }

    @Override
    protected void geometryChangedAction() {
      super.geometryChangedAction();
      this.length = super.getLength();
    }

    @Override
    public double getLength() {
      return this.length;
    }
  }

  public class StrictLineStringGraphGenerator extends
      DirectedLineStringGraphGenerator {

    public StrictLineStringGraphGenerator() {
      super();
      this.setGraphBuilder(new StrictDirectedGraphBuilder());
    }
  }

  public static class VehicleStateAStarFunction extends
      AStarFunctions {

    final double obsStdDevDistance;
    final Coordinate toCoord;

    public VehicleStateAStarFunction(Node destination,
      Coordinate toCoord, double obsStdDevDistance) {
      super(destination);
      this.toCoord = toCoord;
      this.obsStdDevDistance = obsStdDevDistance;
    }

    @Override
    public double cost(AStarNode n1, AStarNode n2) {
      final BasicDirectedNode dn1 = (BasicDirectedNode) n1.getNode();
      final BasicDirectedNode dn2 = (BasicDirectedNode) n2.getNode();

      final Edge edgeBetween = dn1.getOutEdge(dn2);
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

    @Override
    public double h(Node n) {

      final double distance =
          ((Point) n.getObject()).getCoordinate().distance(
              this.toCoord);

      if (distance < this.obsStdDevDistance) {
        return 0d;
      }

      return (distance - this.obsStdDevDistance) / 15d; // 15 m/s, ~35 mph, a random driving speed
    }
  }

  protected static final Logger log = LoggerFactory
      .getLogger(GenericJTSGraph.class);

  protected static final double MAX_DISTANCE_SPEED = 53.6448; // ~120 mph

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

  protected STRtree edgeIndex = null;

  Map<Edge, InferenceGraphEdge> edgeToInfEdge = Maps.newHashMap();
  protected Envelope gpsEnv = null;

  protected DirectedLineStringGraphGenerator graphGenerator = null;

  Map<String, InferenceGraphEdge> idToInfEdge = Maps.newHashMap();

  protected Envelope projEnv = null;

  protected GenericJTSGraph() {
  }

  /**
   * The given collection of lines should be in GPS coordinates, since they will
   * be projected here.
   * 
   * @param lines
   */
  public GenericJTSGraph(Collection<LineString> lines) {
    this.createGraphFromLineStrings(lines);
  }

  protected void createGraphFromLineStrings(
    Collection<LineString> lines) {
    this.graphGenerator = new StrictLineStringGraphGenerator();
    this.edgeIndex = new STRtree();
    this.gpsEnv = new Envelope();
    this.projEnv = new Envelope();
    for (final LineString edge : lines) {
      this.gpsEnv.expandToInclude(edge.getEnvelopeInternal());
      final MathTransform transform =
          GeoUtils.getTransform(edge.getCoordinate());
      Geometry projectedEdge;
      try {
        projectedEdge = JTS.transform(edge, transform);
        this.projEnv.expandToInclude(projectedEdge
            .getEnvelopeInternal());
        final ConstLineString constLine =
            new ConstLineString((LineString) projectedEdge);
        constLine.setUserData(edge);
        this.graphGenerator.add(constLine);
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
    for (final Object obj : this.graphGenerator.getGraph().getEdges()) {
      final BasicDirectedEdge edge = (BasicDirectedEdge) obj;
      final InferenceGraphEdge infEdge =
          this.getInferenceGraphEdge(edge);
      final LineString lineString =
          (LineString) infEdge.getGeometry();
      for (final LineSegment line : GeoUtils
          .getSubLineSegments(lineString)) {
        final Pair<LineSegment, InferenceGraphEdge> lineEdgePair =
            DefaultPair.create(line, infEdge);
        this.edgeIndex.insert(new Envelope(line.p0, line.p1),
            lineEdgePair);
      }

    }
    this.edgeIndex.build();
  }

  @Override
  public boolean edgeHasReverse(Geometry edge) {
    return this.graphGenerator.get(edge.reverse()) != null;
  }

  @Override
  public Envelope getGPSGraphExtent() {
    return this.gpsEnv;
  }

  @Override
  public Collection<InferenceGraphEdge> getIncomingTransferableEdges(
    InferenceGraphEdge infEdge) {

    final DirectedEdge edge = (DirectedEdge) infEdge.getBackingEdge();
    final Collection<DirectedEdge> inEdges =
        edge.getInNode().getInEdges();

    final Set<InferenceGraphEdge> result = Sets.newHashSet();
    for (final DirectedEdge inEdge : inEdges) {
      result.add(this.getInferenceGraphEdge(inEdge));
    }

    return result;
  }

  private InferenceGraphEdge getInferenceGraphEdge(Edge edge) {
    InferenceGraphEdge infEdge = this.edgeToInfEdge.get(edge);

    if (infEdge == null) {
      final Geometry edgeGeom =
          Preconditions.checkNotNull((Geometry) edge.getObject());
      final int id = edge.getID();
      infEdge = new InferenceGraphEdge(edgeGeom, edge, id, this);

      this.edgeToInfEdge.put(edge, infEdge);
      this.idToInfEdge.put(infEdge.getEdgeId(), infEdge);
    }

    return infEdge;
  }

  public Collection<InferenceGraphEdge> getNearbyEdges(
    Coordinate toCoord, double radius) {
    final Envelope toEnv = new Envelope(toCoord);
    toEnv.expandBy(radius);
    final Set<InferenceGraphEdge> streetEdges = Sets.newHashSet();
    for (final Object obj : this.edgeIndex.query(toEnv)) {
      final Pair<LineSegment, InferenceGraphEdge> lineEdgePair =
          (Pair<LineSegment, InferenceGraphEdge>) obj;
      if (lineEdgePair.getFirst().distance(toCoord) < radius) {
        streetEdges.add(lineEdgePair.getSecond());
      } else {
        continue;
      }
    }
    return streetEdges;
  }

  @Override
  public Collection<InferenceGraphEdge> getNearbyEdges(
    DistributionWithMean<Vector> initialBelief, Matrix covar) {

    Preconditions.checkArgument(initialBelief.getMean()
        .getDimensionality() == 4);

    final Vector toLoc =
        MotionStateEstimatorPredictor.getOg().times(
            initialBelief.getMean());
    final double varDistance =
        StatisticsUtil.getLargeNormalCovRadius((DenseMatrix) covar);

    return this.getNearbyEdges(toLoc, varDistance);
  }

  @Override
  public Collection<InferenceGraphEdge> getNearbyEdges(
    Vector projLocation, double radius) {
    Preconditions
        .checkArgument(projLocation.getDimensionality() == 2);
    return this.getNearbyEdges(GeoUtils.makeCoordinate(projLocation),
        radius);
  }

  @Override
  public Collection<InferenceGraphEdge> getOutgoingTransferableEdges(
    InferenceGraphEdge infEdge) {

    final DirectedEdge edge = (DirectedEdge) infEdge.getBackingEdge();
    final Collection<DirectedEdge> outEdges =
        edge.getOutNode().getOutEdges();

    final Set<InferenceGraphEdge> result = Sets.newHashSet();
    for (final DirectedEdge outEdge : outEdges) {
      result.add(this.getInferenceGraphEdge(outEdge));
    }

    return result;
  }

  @Override
  public Set<Path> getPaths(final VehicleState<?> fromState,
    final GpsObservation obs) {

    final Coordinate toCoord = obs.getObsProjected();

    final InferenceGraphEdge currentEdge =
        fromState.getPathStateParam().getValue().getEdge()
            .getInferredEdge();

    final Set<InferenceGraphEdge> startEdges = Sets.newHashSet();

    if (!currentEdge.isNullEdge()) {
      startEdges.add(currentEdge);
    } else {

      final MultivariateGaussian obsBelief =
          fromState.getMotionStateEstimatorPredictor()
              .getMeasurementBelief(
                  fromState.getPathStateParam().getParameterPrior());

      final double beliefDistance =
          Math.min(StatisticsUtil
              .getLargeNormalCovRadius((DenseMatrix) obsBelief
                  .getCovariance()),
              GenericJTSGraph.MAX_STATE_SNAP_RADIUS);

      startEdges.addAll(this.getNearbyEdges(
          fromState.getMeanLocation(), beliefDistance));
    }

    final double obsStdDevDistance =
        Math.min(StatisticsUtil
            .getLargeNormalCovRadius((DenseMatrix) fromState
                .getObservationCovarianceParam().getValue()),
            GenericJTSGraph.MAX_OBS_SNAP_RADIUS);

    final Set<Node> endNodes = Sets.newHashSet();
    for (final InferenceGraphEdge edge : this.getNearbyEdges(toCoord,
        obsStdDevDistance)) {
      final DirectedEdge bEdge =
          ((DirectedEdge) edge.getBackingEdge());
      endNodes.add(bEdge.getNodeA());
      endNodes.add(bEdge.getNodeB());
    }

    final Set<Path> paths = Sets.newHashSet();
    paths.add(new Path());

    if (endNodes.isEmpty()) {
      return paths;
    }

    for (final InferenceGraphEdge startEdge : startEdges) {

      // TODO FIXME determine when/how backward movement fits in
      paths.add(new Path(new PathEdge<InferenceGraphEdge>(startEdge,
          0d, false)));

      final DirectedEdge bStartEdge =
          ((DirectedEdge) startEdge.getBackingEdge());
      final Node source = bStartEdge.getOutNode();

      /*
       * Use this set to avoid recomputing subpaths of 
       * our current paths.
       */
      final Set<Node> reachedEndNodes = Sets.newHashSet(source);
      for (final Node target : endNodes) {

        if (reachedEndNodes.contains(target)) {
          continue;
        }

        final AStarFunctions afuncs =
            new VehicleStateAStarFunction(target, toCoord,
                obsStdDevDistance);

        final CustomAStarShortestPathFinder aStarIter =
            new CustomAStarShortestPathFinder(
                this.graphGenerator.getGraph(), source, target,
                afuncs);
        aStarIter.calculate();

        final org.geotools.graph.path.Path path = aStarIter.getPath();

        if (path != null) {
          final List<PathEdge<?>> pathEdges = Lists.newArrayList();
          double distToStart = 0d;
          final Iterator<?> iter = path.riterator();
          BasicDirectedNode prevNode =
              (BasicDirectedNode) bStartEdge.getInNode();
          while (iter.hasNext()) {
            final BasicDirectedNode node =
                (BasicDirectedNode) iter.next();
            final Edge edge =
                Preconditions.checkNotNull(prevNode.getOutEdge(node));

            final InferenceGraphEdge infEdge =
                this.getInferenceGraphEdge(edge);
            pathEdges.add(new PathEdge<InferenceGraphEdge>(infEdge,
                distToStart, false));
            distToStart += infEdge.getLength();

            reachedEndNodes.add(node);
            prevNode = node;
          }
          if (!pathEdges.isEmpty()) {
            paths.add(new Path(pathEdges, false));
          }
        }

        // TODO backward paths? 

      }
    }

    // TODO debug; remove.
    //    if (obs instanceof TrueObservation) {
    //      final VehicleState trueState = ((TrueObservation)obs).getTrueState();
    //      if (!trueState.getBelief().getPath().isNullPath() && 
    //          fromState.getBelief().getEdge().getInferenceGraphEdge()
    //            .equals(
    //                Iterables.getFirst(trueState.getBelief().getPath().getPathEdges(), null).
    //                getInferenceGraphEdge())
    //            && Iterables.find(paths, 
    //                new Predicate<Path>() {
    //
    //                  @Override
    //                  public boolean apply(Path input) {
    //                    return input.getGeometry() != null &&
    //                        input.getGeometry().covers(
    //                          trueState.getBelief().getPath().getGeometry());
    //                  }
    //                }
    //                , null) == null) {
    //        log.warn("True path not found in search results: true=" 
    //                + trueState.getBelief().getPath() + ", found=" + paths);
    //      }
    //    }

    return paths;
  }

  @Override
  public Envelope getProjGraphExtent() {
    return this.projEnv;
  }

  @Override
  public Set<InferenceGraphEdge> getTopoEquivEdges(
    InferenceGraphEdge edge) {
    final Set<InferenceGraphEdge> result = Sets.newHashSet();
    /*
     * Get reverse edge, if it's there
     */
    final Coordinate[] coords = edge.getGeometry().getCoordinates();
    final Edge revEdge =
        this.graphGenerator.getEdge(coords[coords.length - 1],
            coords[0]);

    final InferenceGraphEdge revInfEdge =
        this.getInferenceGraphEdge(revEdge);
    result.add(edge);
    result.add(revInfEdge);

    return result;
  }

}
