package org.openplans.tools.tracking.impl.util;

import java.io.File;
import java.util.List;
import java.util.Set;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.SnappedEdges;
import org.opentripplanner.graph_builder.impl.map.StreetMatcher;
import org.opentripplanner.model.GraphBundle;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseOptions;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.GraphServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl;
import org.opentripplanner.routing.location.StreetLocation;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableList.Builder;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.Geometry;

public class OtpGraph {

  private static final Logger log = LoggerFactory
      .getLogger(OtpGraph.class);

  private final GraphServiceImpl gs;
  private final GraphBundle bundle;

  private final Graph graph;

  private final StreetVertexIndexServiceImpl indexService;

  private final static TraverseOptions defaultOptions = new TraverseOptions(
      TraverseMode.CAR);

  private final StreetMatcher streetMatcher;

  public OtpGraph() {
    log.info("Loading OTP graph...");

    gs = new GraphServiceImpl();

    bundle = new GraphBundle(new File(
        "../src/main/resources/org/openplans/cebutaxi/"));

    gs.setBundle(bundle);
    gs.refreshGraph();

    graph = gs.getGraph();

    streetMatcher = new StreetMatcher(graph);
    indexService = new StreetVertexIndexServiceImpl(graph);
    indexService.setup();

    log.info("Graph loaded..");
  }

  public GraphBundle getBundle() {
    return bundle;
  }

  public Graph getGraph() {
    return graph;
  }

  public GraphServiceImpl getGs() {
    return gs;
  }

  public StreetVertexIndexServiceImpl getIndexService() {
    return indexService;
  }

  public TraverseOptions getOptions() {
    return defaultOptions;
  }

  public StreetMatcher getStreetMatcher() {
    return streetMatcher;
  }

  public int getVertexCount() {
    return graph.getVertices().size();
  }

  /**
   * Snaps the observed location to a graph edge, computes edges traveled
   * between observations (when applicable), and returns both sets of edges.
   * 
   * @param loc
   * @return
   */
  public SnappedEdges snapToGraph(Coordinate fromCoords, Coordinate toCoords) {

    Preconditions.checkNotNull(toCoords);

    final TraverseOptions options = OtpGraph.defaultOptions;
    final Vertex snappedVertex = indexService.getClosestVertex(toCoords, null,
        options);
    final Builder<Edge> pathTraversed = ImmutableList.builder();
    final com.google.common.collect.ImmutableSet.Builder<Edge> snappedEdges = ImmutableSet
        .<Edge> builder();
    if (snappedVertex != null && (snappedVertex instanceof StreetLocation)) {

      final StreetLocation snappedStreetLocation = (StreetLocation) snappedVertex;
      // final double dist = snappedVertex.distance(obsCoords);

      /*
       * Just find the edge for the isolate point
       */
      final Set<Edge> edges = Sets.newHashSet();
      edges.addAll(Objects.firstNonNull(
          snappedStreetLocation.getOutgoingStreetEdges(),
          ImmutableList.<Edge> of()));
      edges.addAll(Objects.firstNonNull(snappedStreetLocation.getIncoming(),
          ImmutableList.<Edge> of()));

      for (final Edge edge : Objects.firstNonNull(
          snappedStreetLocation.getOutgoingStreetEdges(),
          ImmutableList.<Edge> of())) {
        snappedEdges.add(edge);
      }

      if (fromCoords != null && !fromCoords.equals2D(toCoords)) {
        final CoordinateSequence movementSeq = JTSFactoryFinder
            .getGeometryFactory().getCoordinateSequenceFactory()
            .create(new Coordinate[] { fromCoords, toCoords });
        final Geometry movementGeometry = JTSFactoryFinder.getGeometryFactory()
            .createLineString(movementSeq);

        /*
         * Find the edges between the two observed points.
         */
        final List<Edge> minimumConnectingEdges = Objects.firstNonNull(
            streetMatcher.match(movementGeometry), ImmutableList.<Edge> of());

        for (final Edge edge : minimumConnectingEdges) {
          pathTraversed.add(edge);
        }

      } else {

      }
    }
    return new SnappedEdges(snappedEdges.build(), pathTraversed.build());
  }

}
