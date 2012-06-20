package org.openplans.tools.tracking.impl.util;

import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.opentripplanner.graph_builder.impl.map.StreetMatcher;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.OutEdge;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.edgetype.TurnEdge;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Graph.LoadLevel;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.GraphServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl.CandidateEdgeBundle;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.ApplicationContext;
import org.springframework.context.support.GenericApplicationContext;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.index.strtree.STRtree;

public class OtpGraph {

  private static final Logger log = LoggerFactory
      .getLogger(OtpGraph.class);

  private final GraphServiceImpl gs;

  private final Graph graph;

  private final StreetVertexIndexServiceImpl indexService;

  private final static RoutingRequest defaultOptions = new RoutingRequest(
      TraverseMode.CAR);

  private final StreetMatcher streetMatcher;
  
  private STRtree edgeIndex = new STRtree();
  private STRtree vertexIndex = new STRtree();
  
  private void createIndices() {

    for (final Vertex v : graph.getVertices()) {

      final Geometry vertexGeometry = GeoUtils.lonlatToGeometry(v
          .getCoordinate());
      final Envelope vertexEnvelope = vertexGeometry.getEnvelopeInternal();
      vertexIndex.insert(vertexEnvelope, v);

      for (final Edge e : v.getOutgoing()) {
        if (graph.getIdForEdge(e) != null) {
          final Geometry geometry = e.getGeometry();
          final Envelope envelope = geometry.getEnvelopeInternal();
          edgeIndex.insert(envelope, e);
        }
      }
    }
  }
  

  public OtpGraph(String path) {
    log.info("Loading OTP graph...");

    gs = new GraphServiceImpl();
    gs.setLoadLevel(LoadLevel.DEBUG);

    final ApplicationContext appContext = new GenericApplicationContext();

    gs.setResourceLoader(appContext);

    gs.setPath(path);
    gs.refreshGraphs();

    graph = gs.getGraph();

    streetMatcher = new StreetMatcher(graph);
    indexService = new StreetVertexIndexServiceImpl(graph);
    indexService.setup();
    createIndices();

    log.info("Graph loaded..");
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

  public RoutingRequest getOptions() {
    return defaultOptions;
  }

  public StreetMatcher getStreetMatcher() {
    return streetMatcher;
  }

  public int getVertexCount() {
    return graph.getVertices().size();
  }

  public List<StreetEdge> snapToGraph(Coordinate toCoords) {

    Preconditions.checkNotNull(toCoords);

    final RoutingRequest options = OtpGraph.defaultOptions;
    /*
     * XXX: indexService uses lon/lat
     */
    final CandidateEdgeBundle edgeBundle = indexService
        .getClosestEdges(
            new Coordinate(toCoords.y, toCoords.x), options, null,
            null);
    return edgeBundle.toEdgeList();
  }

  public static List<Edge> filterForStreetEdges(Collection<Edge> edges) {
    final List<Edge> result = Lists.newArrayList();
    for (final Edge out : edges) {
      if (!(out instanceof TurnEdge || out instanceof OutEdge || out instanceof PlainStreetEdge)) {
        continue;
      }
      result.add(out);
    }
    return result;
  }

  public static boolean isStreetEdge(Edge pathEdge) {
    if (!(pathEdge instanceof TurnEdge || pathEdge instanceof OutEdge || pathEdge instanceof PlainStreetEdge))
      return false;
    else
      return true;
  }


  public STRtree getEdgeIndex() {
    return edgeIndex;
  }


  public STRtree getVertexIndex() {
    return vertexIndex;
  }

}
