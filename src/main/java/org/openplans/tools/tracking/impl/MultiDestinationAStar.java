package org.openplans.tools.tracking.impl;

import java.util.List;

import org.opentripplanner.common.geometry.DistanceLibrary;
import org.opentripplanner.routing.algorithm.GenericAStar;
import org.opentripplanner.routing.algorithm.strategies.RemainingWeightHeuristic;
import org.opentripplanner.routing.algorithm.strategies.SearchTerminationStrategy;
import org.opentripplanner.routing.algorithm.strategies.SkipTraverseResultStrategy;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.opentripplanner.routing.vertextype.IntersectionVertex;

import com.vividsolutions.jts.geom.Coordinate;

public class MultiDestinationAStar implements
    SearchTerminationStrategy, RemainingWeightHeuristic,
    SkipTraverseResultStrategy {
  private static final long serialVersionUID = 1L;

  double MAX_SPEED = 27.0; // ~60 mph

  private final List<Edge> end;
  private final Coordinate center;
  private final double radius;
  private final Graph graph;

  private final Edge start;

  MultiDestinationAStar(Graph graph, List<Edge> end,
    Coordinate center, double radius, Edge start) {
    this.graph = graph;
    this.end = end;
    this.center = center;
    this.radius = radius;
    this.start = start;
  }

  @Override
  public double computeForwardWeight(State s, Vertex target) {
    final Vertex v = s.getVertex();
    final double distance = DistanceLibrary.fastDistance(
        v.getCoordinate(), center)
        - radius;

    return distance / MAX_SPEED;
  }

  @Override
  public double computeInitialWeight(State s, Vertex target) {
    return computeForwardWeight(s, target);
  }

  @Override
  public double computeReverseWeight(State s, Vertex target) {
    return computeForwardWeight(s, target);
  }

  ShortestPathTree getSPT(boolean arriveBy) {
    // set up
    final GenericAStar astar = new GenericAStar();
    astar.setSearchTerminationStrategy(this);
    astar.setSkipTraverseResultStrategy(this);
    final RoutingRequest req = new RoutingRequest();
    final Vertex startVertex = arriveBy ? start.getToVertex() : start.getFromVertex();
    // TODO FIXME how do we really avoid the name collisions?
    final String bogusName = "bogus" + System.nanoTime();
    final Vertex bogus = new IntersectionVertex(graph, bogusName, 
        startVertex.getCoordinate(), bogusName);
    req.setRoutingContext(graph, startVertex, bogus);
    req.rctx.remainingWeightHeuristic = this;
    req.setArriveBy(arriveBy);
    final ShortestPathTree result = astar.getShortestPathTree(req);
    graph.removeVertex(bogus);
    req.cleanup();
    return result;

  }

  @Override
  public void reset() {
    // not implemented because it is unused, unfortunately
  }

  @Override
  public boolean shouldSearchContinue(Vertex origin, Vertex target,
    State current, ShortestPathTree spt,
    RoutingRequest traverseOptions) {
    end.remove(current.getBackEdge());
    return end.size() != 0;
  }

  @Override
  public boolean shouldSkipTraversalResult(Vertex origin,
    Vertex target, State parent, State current, ShortestPathTree spt,
    RoutingRequest traverseOptions) {
    if (parent.getVertex() == origin
        && current.getBackEdge() != start) {
      return true;
    }
    return false;
  }
}