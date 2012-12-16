package org.openplans.tools.tracking.impl.statistics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.util.OtpGraph;
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

  @Before
  public void testSetup() {
    
  }
  
  @Test
  public void testUnique() {
    Graph graph = new Graph();

    StreetVertex v0 = new IntersectionVertex(graph, "v0", 0, 1);
    StreetVertex v1 = new IntersectionVertex(graph, "v1", 1, 1.1);
    StreetVertex v2 = new IntersectionVertex(graph, "v2", 2, 1.2);
    StreetVertex v3 = new IntersectionVertex(graph, "v3", 2, 1.221);
    StreetVertex v4 = new IntersectionVertex(graph, "v4", 2, 1.222);
    StreetVertex v5 = new IntersectionVertex(graph, "v5", 3, 1.2221);

    Edge e1 = new PlainStreetEdge(
        v0, v1, makeGeometry(v0, v1), "e1", 1.0,
        StreetTraversalPermission.ALL, false);
    Edge e2 = new PlainStreetEdge(
        v0, v2, makeGeometry(v0, v2), "e2", 1.0,
        StreetTraversalPermission.ALL, false);
    Edge e3 = new PlainStreetEdge(
        v2, v3, makeGeometry(v2, v3), "e3", 1.0,
        StreetTraversalPermission.ALL, false);
    Edge e4 = new PlainStreetEdge(
        v2, v4, makeGeometry(v2, v4), "e4", 1.0,
        StreetTraversalPermission.ALL, false);
    Edge e5 = new PlainStreetEdge(
        v3, v5, makeGeometry(v3, v5), "e5", 1.0,
        StreetTraversalPermission.ALL, false);

    Set<InferredPath> paths = new HashSet<InferredPath>();

    OtpGraph otpGraph = mock(OtpGraph.class);
    
    List<PathEdge> edges1 = new ArrayList<PathEdge>(
        Arrays.asList(PathEdge.getEdge(new InferredEdge(
            e1, 1, otpGraph), 0, false)));
    InferredPath p1 = InferredPath.getInferredPath(edges1, false);
    paths.add(p1);

    //a single path should be passed unchanged.
    OtpGraph.makeUnique(paths);
    assertEquals(1, paths.size());

    List<PathEdge> edges2 = Arrays.asList(PathEdge.getEdge(
        new InferredEdge(e2, 2, otpGraph), 0, false));
    InferredPath p2 = InferredPath.getInferredPath(edges2, false);
    paths.add(p2);

    //two non-overlapping paths should not be compressed
    OtpGraph.makeUnique(paths);
    assertEquals(2, paths.size());

    List<PathEdge> edges3 = Arrays.asList(
        PathEdge.getEdge(new InferredEdge(e2, 2, otpGraph), 0, false),
        PathEdge.getEdge(new InferredEdge(e3, 3, otpGraph), 0, false));
    InferredPath p3 = InferredPath.getInferredPath(edges3, false);
    paths.add(p3);

    //an overlapping path should supersede a smaller one
    OtpGraph.makeUnique(paths);
    assertEquals(2, paths.size());
    assertTrue(paths.contains(p3));

    //the most complicated example
    List<PathEdge> edges4 = Arrays.asList(
        PathEdge.getEdge(new InferredEdge(e2, 2, otpGraph), 0, false),
        PathEdge.getEdge(new InferredEdge(e4, 4, otpGraph), 0, false));
    InferredPath p4 = InferredPath.getInferredPath(edges4, false);
    paths.add(p4);

    List<PathEdge> edges5 = Arrays.asList(
        PathEdge.getEdge(new InferredEdge(e2, 2, otpGraph), 0, false),
        PathEdge.getEdge(new InferredEdge(e4, 4, otpGraph), 0, false),
        PathEdge.getEdge(new InferredEdge(e5, 5, otpGraph), 0, false));
    InferredPath p5 = InferredPath.getInferredPath(edges5, false);
    paths.add(p5);
    OtpGraph.makeUnique(paths);
    assertEquals(3, paths.size());
    assertTrue(paths.contains(p5));
    assertTrue(paths.contains(p3));
    assertTrue(paths.contains(p1));
  }

  private LineString makeGeometry(StreetVertex v0, StreetVertex v1) {
    GeometryFactory gf = new GeometryFactory();
    Coordinate[] coordinates = new Coordinate[] { v0.getCoordinate(),
        v1.getCoordinate() };
    return gf.createLineString(coordinates);
  }
}
