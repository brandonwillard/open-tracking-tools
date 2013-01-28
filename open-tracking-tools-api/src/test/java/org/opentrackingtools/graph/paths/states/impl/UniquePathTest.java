package org.opentrackingtools.graph.paths.states.impl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.Before;
import org.junit.Test;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.vertextype.IntersectionVertex;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;

public class UniquePathTest {

  private LineString makeGeometry(StreetVertex v0,
    StreetVertex v1) {
    final GeometryFactory gf = new GeometryFactory();
    final Coordinate[] coordinates =
        new Coordinate[] { v0.getCoordinate(),
            v1.getCoordinate() };
    return gf.createLineString(coordinates);
  }

  @Before
  public void testSetup() {

  }

  @Test
  public void testUnique() {
    final Graph graph = new Graph();

    final StreetVertex v0 =
        new IntersectionVertex(graph, "v0", 0, 1);
    final StreetVertex v1 =
        new IntersectionVertex(graph, "v1", 1, 1.1);
    final StreetVertex v2 =
        new IntersectionVertex(graph, "v2", 2, 1.2);
    final StreetVertex v3 =
        new IntersectionVertex(graph, "v3", 2, 1.221);
    final StreetVertex v4 =
        new IntersectionVertex(graph, "v4", 2, 1.222);
    final StreetVertex v5 =
        new IntersectionVertex(graph, "v5", 3, 1.2221);

    final Edge e1 =
        new PlainStreetEdge(v0, v1, makeGeometry(v0, v1),
            "e1", 1.0, StreetTraversalPermission.ALL, false);
    final Edge e2 =
        new PlainStreetEdge(v0, v2, makeGeometry(v0, v2),
            "e2", 1.0, StreetTraversalPermission.ALL, false);
    final Edge e3 =
        new PlainStreetEdge(v2, v3, makeGeometry(v2, v3),
            "e3", 1.0, StreetTraversalPermission.ALL, false);
    final Edge e4 =
        new PlainStreetEdge(v2, v4, makeGeometry(v2, v4),
            "e4", 1.0, StreetTraversalPermission.ALL, false);
    final Edge e5 =
        new PlainStreetEdge(v3, v5, makeGeometry(v3, v5),
            "e5", 1.0, StreetTraversalPermission.ALL, false);

    final Set<SimpleInferredPath> paths =
        new HashSet<SimpleInferredPath>();

    final OtpGraph otpGraph = mock(OtpGraph.class);

    final List<SimplePathEdge> edges1 =
        new ArrayList<SimplePathEdge>(Arrays.asList(SimplePathEdge
            .getEdge(SimpleInferredEdge.getInferredEdge(e1.getGeometry(), e1, 1, otpGraph), 0,
                false)));
    final SimpleInferredPath p1 =
        SimpleInferredPath.getInferredPath(edges1, false);
    paths.add(p1);

    //a single path should be passed unchanged.
    OtpGraph.makeUnique(paths);
    assertEquals(1, paths.size());

    final List<SimplePathEdge> edges2 =
        Arrays.asList(SimplePathEdge.getEdge(SimpleInferredEdge.getInferredEdge(
            e2.getGeometry(), e2, 2, otpGraph), 0, false));
    final SimpleInferredPath p2 =
        SimpleInferredPath.getInferredPath(edges2, false);
    paths.add(p2);

    //two non-overlapping paths should not be compressed
    OtpGraph.makeUnique(paths);
    assertEquals(2, paths.size());

    final List<SimplePathEdge> edges3 =
        Arrays.asList(SimplePathEdge.getEdge(SimpleInferredEdge.getInferredEdge(
            e2.getGeometry(), e2, 2, otpGraph), 0, false), SimplePathEdge.getEdge(
            SimpleInferredEdge.getInferredEdge(
                e3.getGeometry(), e3, 3, otpGraph), 0, false));
    final SimpleInferredPath p3 =
        SimpleInferredPath.getInferredPath(edges3, false);
    paths.add(p3);

    //an overlapping path should supersede a smaller one
    OtpGraph.makeUnique(paths);
    assertEquals(2, paths.size());
    assertTrue(paths.contains(p3));

    //the most complicated example
    final List<SimplePathEdge> edges4 =
        Arrays.asList(SimplePathEdge.getEdge(SimpleInferredEdge.getInferredEdge(
            e2.getGeometry(), e2, 2, otpGraph), 0, false), SimplePathEdge.getEdge(
            SimpleInferredEdge.getInferredEdge(
                e4.getGeometry(), e4, 4, otpGraph), 0, false));
    final SimpleInferredPath p4 =
        SimpleInferredPath.getInferredPath(edges4, false);
    paths.add(p4);

    // TODO why doesn't this path make sense?
//    final List<SimplePathEdge> edges5 =
//        Arrays.asList(SimplePathEdge.getEdge(SimpleInferredEdge.getInferredEdge(
//            e2.getGeometry(), e2, 2, otpGraph), 0, false), SimplePathEdge.getEdge(
//            SimpleInferredEdge.getInferredEdge(
//                e4.getGeometry(), e4, 4, otpGraph), 0, false),
//            SimplePathEdge.getEdge(SimpleInferredEdge.getInferredEdge(e5.getGeometry(), 
//                e5, 5, otpGraph), 0, false));
//    final SimpleInferredPath p5 =
//        SimpleInferredPath.getInferredPath(edges5, false);
//    paths.add(p5);
//    OtpGraph.makeUnique(paths);
//    assertEquals(3, paths.size());
//    assertTrue(paths.contains(p5));
//    assertTrue(paths.contains(p3));
//    assertTrue(paths.contains(p1));
  }
}
