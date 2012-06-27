/* This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public License
 as published by the Free Software Foundation, either version 3 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

package org.openplans.tools.tracking.impl.graph.paths.algorithms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.common.geometry.DistanceLibrary;
import org.opentripplanner.common.geometry.SphericalDistanceLibrary;
import org.opentripplanner.common.pqueue.BinHeap;
import org.opentripplanner.graph_builder.impl.map.EndMatchState;
import org.opentripplanner.graph_builder.impl.map.MatchState;
import org.opentripplanner.graph_builder.impl.map.MidblockMatchState;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.index.strtree.STRtree;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class PathSampler {
  private static final Logger log = LoggerFactory
      .getLogger(PathSampler.class);
  private static final double DEFAULT_DISTANCE_THRESHOLD = GeoUtils
      .getMetersInAngleDegrees(100); // 0.002;

  private final Graph graph;

  private final STRtree edgeIndex;
  private final STRtree vertexIndex;
  
  private final DistanceLibrary distanceLibrary = SphericalDistanceLibrary.getInstance();

  public PathSampler(Graph graph) {
    this.graph = graph;
    final Entry<STRtree, STRtree> indices = createIndices();
    edgeIndex = indices.getKey();
    vertexIndex = indices.getValue();

    edgeIndex.build();
    vertexIndex.build();
  }

  Entry<STRtree, STRtree> createIndices() {
    final STRtree edgeIndex = new STRtree();
    final STRtree vertexIndex = new STRtree();

    final Map<Geometry, Edge> geoToEdge = Maps.newHashMap();

    for (final Vertex v : graph.getVertices()) {

      // TODO just use the constructor for envelope, no?
      Envelope vertexEnvelope;
      final Geometry vertexGeometry = GeoUtils.lonlatToGeometry(v
          .getCoordinate());
      vertexEnvelope = vertexGeometry.getEnvelopeInternal();
      vertexIndex.insert(vertexEnvelope, v);

      for (final Edge e : v.getOutgoing()) {
        if (graph.getIdForEdge(e) != null) {
          Envelope envelope;
          final Geometry geometry = e.getGeometry();
          envelope = geometry.getEnvelopeInternal();
          edgeIndex.insert(envelope, e);

          if (geoToEdge.containsKey(geometry.reverse())) {

          }

          geoToEdge.put(geometry, e);
        }
      }
    }
    log.debug("Created index");
    return Maps.immutableEntry(edgeIndex, vertexIndex);
  }

  public STRtree getEdgeIndex() {
    return edgeIndex;
  }

  public Graph getGraph() {
    return graph;
  }

  public STRtree getVertexIndex() {
    return vertexIndex;
  }

  public List<Edge> match(Edge initialEdge, Geometry routeGeometry) {

    routeGeometry = removeDuplicatePoints(routeGeometry);

    if (routeGeometry == null)
      return null;

    // initial state: start midway along a block.
    final LocationIndexedLine indexedLine = new LocationIndexedLine(
        routeGeometry);

    final LinearLocation startIndex = indexedLine.getStartIndex();

    final Coordinate routeStartCoordinate = startIndex
        .getCoordinate(routeGeometry);

    final BinHeap<MatchState> states = new BinHeap<MatchState>();
    final Geometry edgeGeometry = initialEdge.getGeometry();

    final LocationIndexedLine indexedEdge = new LocationIndexedLine(
        edgeGeometry);
    final LinearLocation initialLocation = indexedEdge
        .project(routeStartCoordinate);

    final double error = distanceLibrary.fastDistance(
        initialLocation.getCoordinate(edgeGeometry),
        routeStartCoordinate);
    final MatchState startState = new MidblockMatchState(
        null, routeGeometry, initialEdge, startIndex,
        initialLocation, error, 0.01);
    states.insert(startState, 0); // make sure all initial states are visited by
                                  // inserting them at 0

    // search for best-matching path
    int seen_count = 0, total = 0;
    final HashSet<MatchState> seen = new HashSet<MatchState>();
    while (!states.empty()) {
      final double k = states.peek_min_key();
      final MatchState state = states.extract_min();
      if (++total % 50000 == 0) {
        log.debug("seen / total: " + seen_count + " / " + total);
      }
      if (seen.contains(state)) {
        ++seen_count;
        continue;
      } else {
        if (k != 0) {
          // but do not mark states as closed if we start at them
          seen.add(state);
        }
      }
      if (state instanceof EndMatchState) {
        return toEdgeList(state);
      }
      for (final MatchState next : state.getNextStates()) {
        if (seen.contains(next)) {
          continue;
        }
        states
            .insert(
                next,
                next.getTotalError() - next.getDistanceAlongRoute());
      }
    }
    return null;
  }

  public Set<List<Edge>> match(Geometry routeGeometry,
    final double startDistanceThreshold) {

    routeGeometry = removeDuplicatePoints(routeGeometry);

    if (routeGeometry == null)
      return null;

    // initial state: start midway along a block.
    final LocationIndexedLine indexedLine = new LocationIndexedLine(
        routeGeometry);

    final LinearLocation startIndex = indexedLine.getStartIndex();

    final Coordinate routeStartCoordinate = startIndex
        .getCoordinate(routeGeometry);
    final Envelope envelope = new Envelope(routeStartCoordinate);
    double localDistanceThreshold = startDistanceThreshold;
    if (localDistanceThreshold > 0d)
      envelope.expandBy(localDistanceThreshold);

    final BinHeap<MatchState> states = new BinHeap<MatchState>();
    List nearbyEdges = edgeIndex.query(envelope);
    if (localDistanceThreshold > 0d) {
      while (nearbyEdges.isEmpty()) {
        envelope.expandBy(localDistanceThreshold);
        localDistanceThreshold *= 2;
        nearbyEdges = edgeIndex.query(envelope);
      }
    }

    // compute initial states
    for (final Object obj : nearbyEdges) {
      final Edge initialEdge = (Edge) obj;
      final Geometry edgeGeometry = initialEdge.getGeometry();

      final LocationIndexedLine indexedEdge = new LocationIndexedLine(
          edgeGeometry);
      final LinearLocation initialLocation = indexedEdge
          .project(routeStartCoordinate);

      final double error = distanceLibrary.fastDistance(
          initialLocation.getCoordinate(edgeGeometry),
          routeStartCoordinate);
      final MatchState state = new MidblockMatchState(
          null, routeGeometry, initialEdge, startIndex,
          initialLocation, error, 0.01);
      states.insert(state, 0); // make sure all initial states are visited by
                               // inserting them at 0
    }

    // search for best-matching path
    int seen_count = 0, total = 0;
    final Set<List<Edge>> results = Sets.newHashSet();
    final HashSet<MatchState> seen = new HashSet<MatchState>();
    while (!states.empty()) {
      final double k = states.peek_min_key();
      final MatchState state = states.extract_min();
      if (++total % 50000 == 0) {
        log.debug("seen / total: " + seen_count + " / " + total);
      }
      if (seen.contains(state)) {
        ++seen_count;
        continue;
      } else {
        if (k != 0) {
          // but do not mark states as closed if we start at them
          seen.add(state);
        }
      }
      if (state instanceof EndMatchState) {
        results.add(toEdgeList(state));
        continue;
      }
      for (final MatchState next : state.getNextStates()) {
        if (seen.contains(next)) {
          continue;
        }
        states
            .insert(
                next,
                next.getTotalError() - next.getDistanceAlongRoute());
      }
    }
    return results;
  }

  private Geometry removeDuplicatePoints(Geometry routeGeometry) {
    final List<Coordinate> coords = new ArrayList<Coordinate>();
    Coordinate last = null;
    for (final Coordinate c : routeGeometry.getCoordinates()) {
      if (!c.equals(last)) {
        last = c;
        coords.add(c);
      }
    }
    if (coords.size() < 2) {
      return null;
    }
    final Coordinate[] coordArray = new Coordinate[coords.size()];
    return routeGeometry.getFactory().createLineString(
        coords.toArray(coordArray));
  }

  public List<Edge> toEdgeList(MatchState next) {
    final ArrayList<Edge> edges = new ArrayList<Edge>();
    Edge lastEdge = null;
    while (next != null) {
      final Edge edge = next.getEdge();
      if (edge != lastEdge) {
        edges.add(edge);
        lastEdge = edge;
      }
      next = next.parent;
    }
    Collections.reverse(edges);
    return edges;
  }
}
