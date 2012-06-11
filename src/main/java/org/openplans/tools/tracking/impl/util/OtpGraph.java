package org.openplans.tools.tracking.impl.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

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
import org.opentripplanner.routing.impl.GraphServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl.CandidateEdgeBundle;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.context.ApplicationContext;
import org.springframework.context.support.GenericApplicationContext;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

public class OtpGraph {

  private static final Logger log = LoggerFactory
      .getLogger(OtpGraph.class);

  private final GraphServiceImpl gs;

  private final Graph graph;

  private final StreetVertexIndexServiceImpl indexService;

  private final static RoutingRequest defaultOptions = new RoutingRequest(
      TraverseMode.CAR);

  private final StreetMatcher streetMatcher;

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

    log.info("Graph loaded..");
  }

  public static List<Edge> filterForStreetEdges(Collection<Edge> edges) {
    List<Edge> result = Lists.newArrayList();
    for (Edge out : edges) {
      if (!(out instanceof TurnEdge || out instanceof OutEdge || out instanceof PlainStreetEdge)) {
        continue;
      }
      result.add((StreetEdge) out);
    }
    return result;
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

  /**
   * Snaps the observed location to a graph edge, computes edges traveled
   * between observations (when applicable), and returns both sets of edges.
   * 
   * @param loc
   * @return
   */
  public List<StreetEdge> snapToGraph(Coordinate fromCoords,
    Coordinate toCoords) {

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

  public static boolean isStreetEdge(Edge pathEdge) {
    if (!(pathEdge instanceof TurnEdge || pathEdge instanceof OutEdge || pathEdge instanceof PlainStreetEdge))
      return true;
    else
      return false;
  }

}
