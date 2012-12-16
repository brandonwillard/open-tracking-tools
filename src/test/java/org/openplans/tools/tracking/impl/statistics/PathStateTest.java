package org.openplans.tools.tracking.impl.statistics;

import static org.junit.Assert.*;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.*;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.openplans.tools.tracking.impl.TrackingTestUtils;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.AbstractPathState;
import org.openplans.tools.tracking.impl.graph.paths.AbstractPathState.PathMergeResults;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathState;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.vertextype.IntersectionVertex;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.thoughtworks.xstream.io.path.Path;
import com.vividsolutions.jts.algorithm.LineIntersector;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays.ForwardComparator;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.index.chain.MonotoneChain;
import com.vividsolutions.jts.index.chain.MonotoneChainBuilder;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;
import com.vividsolutions.jts.noding.SegmentIntersectionDetector;
import com.vividsolutions.jts.operation.linemerge.LineMergeGraph;
import com.vividsolutions.jts.operation.linemerge.LineSequencer;
import com.vividsolutions.jts.operation.overlay.snap.GeometrySnapper;
import com.vividsolutions.jts.operation.predicate.SegmentIntersectionTester;
import com.vividsolutions.jts.planargraph.DirectedEdge;
import com.vividsolutions.jts.planargraph.Node;
import com.vividsolutions.jts.simplify.TopologyPreservingSimplifier;

public class PathStateTest {

  private InferredPath p1;
  private InferredPath p1_neg;
  private InferredPath p2;
  private InferredPath p2_neg;
  private InferredPath p2_rev;
  private InferredEdge ie2_rev;
  private InferredEdge ie3_rev;
  private double _numericError = 1e-4;
  private InferredEdge ie3;
  private InferredEdge ie2;
  private InferredEdge ie1;
  private Graph graph;
  private OtpGraph otpGraph;
  private InferredEdge ie1_rev;

  @Before
  public void testSetup() {
    graph = new Graph();
    /*
     * Create three stacked edges facing up the y-axis, spaced
     * by 10 meters.
     */
    StreetVertex v0 = new IntersectionVertex(graph, "v0", 0, 10);
    StreetVertex v1 = new IntersectionVertex(graph, "v1", 0, 20);
    StreetVertex v2 = new IntersectionVertex(graph, "v2", 0, 30);
    StreetVertex v3 = new IntersectionVertex(graph, "v3", 0, 40);
    
    Edge e1 = new PlainStreetEdge(
        v0, v1, TrackingTestUtils.makeGeometry(v0, v1), "e1", 10d,
        StreetTraversalPermission.ALL, false);
    Edge e2 = new PlainStreetEdge(
        v1, v2, TrackingTestUtils.makeGeometry(v1, v2), "e2", 10d,
        StreetTraversalPermission.ALL, false);
    Edge e3 = new PlainStreetEdge(
        v2, v3, TrackingTestUtils.makeGeometry(v2, v3), "e3", 10d,
        StreetTraversalPermission.ALL, false);
    
    Edge e1_rev = new PlainStreetEdge(
        v1, v0, TrackingTestUtils.makeGeometry(v1, v0), "-e1", 10d,
        StreetTraversalPermission.ALL, false);
    Edge e2_rev = new PlainStreetEdge(
        v2, v1, TrackingTestUtils.makeGeometry(v2, v1), "-e2", 10d,
        StreetTraversalPermission.ALL, false);
    Edge e3_rev = new PlainStreetEdge(
        v3, v2, TrackingTestUtils.makeGeometry(v3, v2), "-e3", 10d,
        StreetTraversalPermission.ALL, false);
    
    otpGraph = mock(OtpGraph.class);
    
    ie1 = new InferredEdge(e1, 1, otpGraph);
    ie2 = new InferredEdge(e2, 2, otpGraph);
    ie3 = new InferredEdge(e3, 3, otpGraph);
    ie1_rev = new InferredEdge(e1_rev, -1, otpGraph);
    ie2_rev = new InferredEdge(e2_rev, -2, otpGraph);
    ie3_rev = new InferredEdge(e3_rev, -3, otpGraph);
    
    List<PathEdge> edges1 = 
        Lists.newArrayList(
            PathEdge.getEdge(ie1, 0, false),
            PathEdge.getEdge(ie2, ie1.getLength(), false));
    
    p1 = InferredPath.getInferredPath(edges1, false);
    
    List<PathEdge> edges2 = 
        Lists.newArrayList(
            PathEdge.getEdge(ie2, 0, false),
            PathEdge.getEdge(ie3, ie2.getLength(), false));
    
    p2 = InferredPath.getInferredPath(edges2, false);
    
    List<PathEdge> edges1_neg = 
        Lists.newArrayList(
            PathEdge.getEdge(ie2, 0, true),
            PathEdge.getEdge(ie1, -ie1.getLength(), true)
            );
    
    p1_neg = InferredPath.getInferredPath(edges1_neg, true);
    
    List<PathEdge> edges2_neg = 
        Lists.newArrayList(
            PathEdge.getEdge(ie3, 0, true),
            PathEdge.getEdge(ie2, -ie2.getLength(), true));
    
    p2_neg = InferredPath.getInferredPath(edges2_neg, true);
    
    List<PathEdge> edges2_rev = 
        Lists.newArrayList(
            PathEdge.getEdge(ie3_rev, 0, false),
            PathEdge.getEdge(ie2_rev, ie2_rev.getLength(), false)
            );
    
    p2_rev = InferredPath.getInferredPath(edges2_rev, false);
    
  }
  
  @Test
  public void testNonSimplePathCombine1() {
    
    /* 
     *         +--+->  path 1 
     *         |  |  
     *  path 2 v  |
     *         ----
     */
    InferredPath path1 = makeTmpPath(false, 
        new Coordinate(0, 10),
        new Coordinate(0, 0),
        new Coordinate(10, 0),
        new Coordinate(10, 10),
        new Coordinate(20, 10));
    
    InferredPath path2 = makeTmpPath(false, 
        new Coordinate(20, 10),
        new Coordinate(10, 10),
        new Coordinate(0, 10),
        new Coordinate(0, 0));
    
    Geometry from = path1.getGeometry();
    Geometry to = path2.getGeometry();
    
    PathMergeResults res1 = AbstractPathState.mergePaths(from,
        35d, to, 35d);

    Geometry expRes1 = JTSFactoryFinder.getGeometryFactory()
        .createLineString(
           new Coordinate[] {
               new Coordinate(0, 10),
               new Coordinate(0, 0),
               new Coordinate(10, 0),
               new Coordinate(10, 10),
               new Coordinate(20, 10),
               new Coordinate(10, 10),
               new Coordinate(0, 10),
               new Coordinate(0, 0)
           });
    
    assertTrue(res1.getPath().equalsExact(expRes1));
    assertTrue(!res1.isToIsReversed());
    
    PathMergeResults res2 = AbstractPathState.mergePaths(from,
        35d, to.reverse(), 5d);
    
    assertTrue(res2.getPath().equalsExact(expRes1));
    assertTrue(res2.isToIsReversed());
    
    PathMergeResults res3 = AbstractPathState.mergePaths(from,
        2.5d, to, 5d);
    
    Geometry expRes3 = JTSFactoryFinder.getGeometryFactory()
        .createLineString(
           new Coordinate[] {
               new Coordinate(0, 10),
               new Coordinate(0, 0),
               new Coordinate(10, 0),
               new Coordinate(10, 10),
               new Coordinate(20, 10),
           });
    
    assertTrue(res3.getPath().equalsExact(expRes3));
    assertTrue(res3.isToIsReversed());
    
    InferredPath path11 = makeTmpPath(false, 
        new Coordinate(10, 10),
        new Coordinate(20, 10),
        new Coordinate(20, 20)
        );
    
    PathMergeResults res4 = AbstractPathState.mergePaths(
        path11.getGeometry(), 17.5d, to, 2.5d);
    
    Geometry expRes2 = JTSFactoryFinder.getGeometryFactory()
        .createLineString(
           new Coordinate[] {
//               new Coordinate(0, 0),
//               new Coordinate(0, 10),
               new Coordinate(10, 10),
               new Coordinate(20, 10),
               new Coordinate(20, 20)
           });
    
    assertTrue(res4.getPath().equalsExact(expRes2));
    assertTrue(res4.isToIsReversed());
    
    PathMergeResults res5 = AbstractPathState.mergePaths(
        path11.getGeometry(), 7.5d, to, 2.5d);
    
    Geometry expRes5 = JTSFactoryFinder.getGeometryFactory()
        .createLineString(
           new Coordinate[] {
               new Coordinate(10, 10),
               new Coordinate(20, 10),
               new Coordinate(20, 20)
//               new Coordinate(10, 10),
//               new Coordinate(0, 10),
//               new Coordinate(0, 0)
           });
    
    assertTrue(res5.getPath().equalsExact(expRes5));
    assertTrue(res5.isToIsReversed());
    
    Geometry p12 = JTSFactoryFinder.getGeometryFactory()
        .createLineString(
           new Coordinate[] {
               new Coordinate(10, 10),
               new Coordinate(20, 10),
               new Coordinate(10, 10),
               new Coordinate(0, 10),
               new Coordinate(0, 0)
           });
    PathMergeResults res6 = AbstractPathState.mergePaths(
        p12, 37.5d, p12, 32.5d);
    
    assertTrue(res6.getPath().equalsExact(p12));
    assertTrue(!res6.isToIsReversed());
    
    PathMergeResults res7 = AbstractPathState.mergePaths(
        p12, 37.5d, p12.reverse(), 32.5d);
    
    assertTrue(res7.getPath().equalsExact(p12));
    assertTrue(res7.isToIsReversed());
  }
  
  @Test(expected=IllegalArgumentException.class)
  public void testBadPathError() {
    List<PathEdge> edges2_rev = 
        Lists.newArrayList(
            PathEdge.getEdge(ie2_rev, 0, false),
            PathEdge.getEdge(ie3_rev, ie2_rev.getLength(), false)
            );
    
    InferredPath.getInferredPath(edges2_rev, false);
    
  }
  
  @Test
  public void testDistanceBetween1() {
    /*
     * States on the same path.
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(7d, 2d));
    Vector diff = y.minus(x);
    assertEquals("distance", 2d, diff.getElement(0), _numericError);
    assertEquals("velocity", 1d, diff.getElement(1), _numericError);
    
    /*
     * Check for symmetry
     */
    Vector diff2 = x.minus(y);
    assertEquals("rev diff", diff.scale(-1), diff2);
  }
  
  @Ignore
  @Test(expected=IllegalStateException.class)
  public void testDistanceBetween2exp() {
    /*
     * TODO: remove or make relevant again.
     * Same paths, different directions.
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(p1_neg, 
        VectorFactory.getDenseDefault().createVector2D(
            p1_neg.getTotalPathDistance()+7d, -2d));
    Vector diff = y.minus(x);
  }
  
  @Test
  public void testDistanceBetween2() {
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(p1_neg, 
        VectorFactory.getDenseDefault().createVector2D(
            p1_neg.getTotalPathDistance()+7d, -2d));
    Vector diff2 = x.minus(y);
    assertEquals("distance", -2d, diff2.getElement(0), _numericError);
    assertEquals("velocity", 3d, diff2.getElement(1), _numericError);
  }
    
  @Test
  public void testDistanceBetween3() {
    /*
     * Both paths are in the same direction, overlapping.
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(12.5d, 1d));
    PathState y = PathState.getPathState(p2, 
        VectorFactory.getDenseDefault().createVector2D(17.5d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", 15d, diff.getElement(0), _numericError);
    assertEquals("velocity", -2d, diff.getElement(1), _numericError);
  }
  
  @Test
  public void testDistanceBetween4() {
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(12.5d, 1d));
    PathState y = PathState.getPathState(
        TrackingTestUtils.makeTmpPath(
            this.otpGraph, true,
            new Coordinate(0, 40),
            new Coordinate(0, 30),
            new Coordinate(0, 20)), 
        VectorFactory.getDenseDefault().createVector2D(-17.5d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", -15d, diff.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff.getElement(1), _numericError);
  }
    
  @Test
  public void testDistanceBetween4_3() {
    PathState x2 = PathState.getPathState(
        TrackingTestUtils.makeTmpPath(
            this.otpGraph, false,
            new Coordinate(0, 30),
            new Coordinate(0, 20),
            new Coordinate(0, 10)), 
        VectorFactory.getDenseDefault().createVector2D(12.5d, 1d));
    PathState y2 = PathState.getPathState(
        TrackingTestUtils.makeTmpPath(
            this.otpGraph, true,
            new Coordinate(0, 40),
            new Coordinate(0, 30),
            new Coordinate(0, 20)), 
        VectorFactory.getDenseDefault().createVector2D(-17.5d, 1d));
    Vector diff2 = y2.minus(x2);
    assertEquals("distance", -20d, diff2.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff2.getElement(1), _numericError);
  }
  
  @Test
  public void testDistanceBetween4_2() {
    PathState x2 = PathState.getPathState(
        TrackingTestUtils.makeTmpPath(
            this.otpGraph, false,
            new Coordinate(0, 30),
            new Coordinate(0, 20),
            new Coordinate(0, 10)), 
        VectorFactory.getDenseDefault().createVector2D(12.5d, 1d));
    PathState y2 = PathState.getPathState(p2, 
        VectorFactory.getDenseDefault().createVector2D(17.5d, 1d));
    Vector diff2 = y2.minus(x2);
    assertEquals("distance", 20d, diff2.getElement(0), _numericError);
    assertEquals("velocity", 2d, diff2.getElement(1), _numericError);
  }
  
  @Ignore
  @Test
  public void testDistanceBetween5() {
    /*
     * FIXME!
     * Both paths are in the same direction, 
     * path2 is the second edge in path1.
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(
        InferredPath.getInferredPath(
            PathEdge.getEdge(p1.getEdges().get(1).getInferredEdge(),
                0d, false)
            ), 
        VectorFactory.getDenseDefault().createVector2D(5d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", 10d, diff.getElement(0), _numericError);
    assertEquals("velocity", -2d, diff.getElement(1), _numericError);
    
    /*
     * Check for symmetry
     */
    Vector diff2 = x.minus(y);
    assertTrue("rev diff", diff2.scale(-1).equals(diff));
  }
  
  @Ignore
  @Test
  public void testDistanceBetween6() {
    /*
     * FIXME!
     * Path2 is the second edge in path1, but
     * opposite direction.
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(
        InferredPath.getInferredPath(
            PathEdge.getEdge(p1.getEdges().get(1).getInferredEdge(),
                0d, true)
            ), 
        VectorFactory.getDenseDefault().createVector2D(0d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", -15d, diff.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff.getElement(1), _numericError);
    
    /*
     * Check for symmetry
     */
    Vector diff2 = x.minus(y);
    assertEquals("rev distance", -15d, diff2.getElement(0), _numericError);
    assertEquals("rev velocity", 0d, diff2.getElement(1), _numericError);
  }
  
  /**
   * Deferring these non-overlapping tests until 
   * we decide to implement this.
   */
  @Ignore
  @Test
  public void testDistanceBetween7() {
    /*
     * Test that edges touching the start or end, same direction, 
     * but not overlapping, are correctly assessed.
     */
    
    /*
     * Touches end
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(
        InferredPath.getInferredPath(
            PathEdge.getEdge(ie3, 0d, false)), 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    Vector diff = y.minus(x);
    assertEquals("distance", 20d, diff.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff.getElement(1), _numericError);
    
    /*
     * Check for symmetry, and that 
     * when something touches the beginning it works
     */
    Vector diff2 = x.minus(y);
    assertTrue("rev diff", diff2.scale(-1).equals(diff));
  }
  
  /**
   * Deferring these non-overlapping tests until 
   * we decide to implement this.
   */
  @Ignore
  @Test
  public void testDistanceBetween8() {
    /*
     * Test that edges touching the start or end, but
     * not overlapping, are correctly assessed, when
     * the paths are opposite directions.
     * 
     * Again, due to the opposite directions, we
     * get the same answer in reverse.
     * (Now commutative)
     */
    
    /*
     * Touches end
     */
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(
        InferredPath.getInferredPath(
            PathEdge.getEdge(ie3_rev, 0d, false)), 
        VectorFactory.getDenseDefault().createVector2D(5d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", -20d, diff.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff.getElement(1), _numericError);
    
    /*
     * Check for symmetry, and that 
     * when something touches the beginning it works
     */
    Vector diff2 = x.minus(y);
    assertEquals("rev distance", -20d, diff2.getElement(0), _numericError);
    assertEquals("rev velocity", 0d, diff2.getElement(1), _numericError);
  }

  /**
   * Deferring this test until we
   * implement the functionality.
   */
  @Ignore
  @Test
  public void testDistanceBetween9() {
    /*
     * Test large overlaps (one edge
     * covers two small edges), 
     * and opposite directions.
     */
    
    final List<Coordinate> p1RevGeomCoords = Lists.newArrayList(p1.getGeometry().reverse().getCoordinates());
    InferredPath pOther = makeTmpPath(false, Iterables.getFirst(p1RevGeomCoords, null), 
        Iterables.getLast(p1RevGeomCoords, null));
    PathState x = PathState.getPathState(p1, 
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    PathState y = PathState.getPathState(pOther,
        VectorFactory.getDenseDefault().createVector2D(5d, -1d));
    Vector diff = y.minus(x);
    assertEquals("distance", -10d, diff.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff.getElement(1), _numericError);
    
    Vector diff2 = x.minus(y);
    assertEquals("rev distance", -10d, diff2.getElement(0), _numericError);
    assertEquals("rev velocity", 0d, diff2.getElement(1), _numericError);
  }
  
  @Ignore
  @Test
  public void testDistanceBetween10() {
    /* FIXME!
     * Test double overlaps: overlaps on the first
     * edge, then doesn't overlap, then overlaps 
     * at the end.
     */
    
    InferredPath path1 = makeTmpPath(false, 
        new Coordinate(0, 10),
        new Coordinate(0, 0),
        new Coordinate(10, 0),
        new Coordinate(10, 10),
        new Coordinate(20, 10));
    
    InferredPath path2 = makeTmpPath(false, 
        new Coordinate(20, 10),
        new Coordinate(10, 10),
        new Coordinate(0, 10),
        new Coordinate(0, 0));
    
    PathState x = PathState.getPathState(path1, 
        VectorFactory.getDenseDefault().createVector2D(35d, 1d));
    PathState y = PathState.getPathState(path2,
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    Vector diff = y.minus(x);
    assertEquals("distance", 0d, diff.getElement(0), _numericError);
    assertEquals("velocity", 2d, diff.getElement(1), _numericError);
    
    Vector diff2 = x.minus(y);
    assertEquals("rev distance", 50d, diff2.getElement(0), _numericError);
    assertEquals("rev velocity", 0d, diff2.getElement(1), _numericError);
    
    PathState x2 = PathState.getPathState(path1, 
        VectorFactory.getDenseDefault().createVector2D(22.5d, 1d));
    PathState y2 = PathState.getPathState(path2,
        VectorFactory.getDenseDefault().createVector2D(5d, 1d));
    Vector diff3 = x2.minus(y2);
    assertEquals("distance", 32.5d, diff3.getElement(0), _numericError);
    assertEquals("velocity", 0d, diff3.getElement(1), _numericError);
  }
  
  @Ignore
  @Test
  public void testDistanceBetween11() {
    /*
     * FIXME!
     */
    
    InferredPath path1 = makeTmpPath(false, 
        new Coordinate(0, 0),
        new Coordinate(0, 10));
    
    InferredPath path2 = makeTmpPath(false, 
        new Coordinate(0, 10),
        new Coordinate(0, 0),
        new Coordinate(-10, 0));
    
    PathState x = PathState.getPathState(path2, 
        VectorFactory.getDenseDefault().createVector2D(15d, 1d));
    PathState y = PathState.getPathState(path1,
        VectorFactory.getDenseDefault().createVector2D(8.5d, 1d));
    Vector diff = y.minus(x);
    assertEquals("distance", -13.5d, diff.getElement(0), _numericError);
    assertEquals("velocity", -2d, diff.getElement(1), _numericError);
    
    Vector diff2 = x.minus(y);
    assertEquals("rev distance", -13.5d, diff2.getElement(0), _numericError);
    assertEquals("rev velocity", -2d, diff2.getElement(1), _numericError);
  }
  
  @Test
  public void testDistanceBetween12() {
    InferredPath startPath = makeTmpPath( 
        false, 
        new Coordinate(0, 0),
        new Coordinate(0, 60)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(12d, -1));
    
    InferredPath newPath = makeTmpPath( 
        true, 
        new Coordinate(0, 60),
        new Coordinate(0, 0)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    PathState stateOnNewPath = PathState.getPathState(newPath, result);
    final Vector difference = stateOnNewPath.minus(
       startState);
    
    assertTrue(difference.isZero(1e-7));
  }
  
  @Test
  public void testDistanceBetween13() {
    InferredPath startPath = makeTmpPath( 
        false, 
        new Coordinate(0, 0),
        new Coordinate(0, 60),
        new Coordinate(0, 199)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(104d, -1));
    
    InferredPath newPath = makeTmpPath( 
        false, 
        new Coordinate(0, 199),
        new Coordinate(0, 60)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    PathState stateOnNewPath = PathState.getPathState(newPath, result);
    final Vector difference = stateOnNewPath.minus(
       startState);
    
    assertTrue(difference.isZero(1e-7));
  }
  
  @Test
  public void testDistanceBetween14() {
    InferredPath startPath = makeTmpPath( 
        true, 
        new Coordinate(0, 199),
        new Coordinate(0, 60),
        new Coordinate(0, 0)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(-104d, -2.79));
    
    InferredPath newPath = makeTmpPath( 
        true, 
        new Coordinate(0, 60),
        new Coordinate(0, 199)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    PathState stateOnNewPath = PathState.getPathState(newPath, result);
    final Vector difference = stateOnNewPath.minus(
       startState);
    
    assertTrue(difference.isZero(1e-7));
  }
  
  @Test
  public void testDistanceBetween15() {
    InferredPath startPath = makeTmpPath( 
        true, 
        new Coordinate(0, 178.89095954422947),
        new Coordinate(0, 0),
        new Coordinate(0, 139.9919743536118d)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(
            -139.9919743536118d, -3.39));
    
    InferredPath newPath = makeTmpPath( 
        false, 
        new Coordinate(0, 0),
        new Coordinate(0, 178.89095954422947)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    PathState stateOnNewPath = PathState.getPathState(newPath, result);
    final Vector difference = stateOnNewPath.minus(
       startState);
    
    assertTrue(difference.isZero(1e-7));
  }
  
  @Test
  public void testDistanceBetween16() {
    InferredPath otherPath = makeTmpPath( 
        false, 
        new Coordinate(0d, 0d),
        new Coordinate(-85d, 0d),
        new Coordinate(-150.37d, 0d)
      );
    
    PathState otherState = PathState.getPathState(otherPath,
        VectorFactory.getDefault().createVector2D(
            121.15d, 2.59d));
    
    InferredPath thisPath = makeTmpPath( 
        true, 
        new Coordinate(0d, 0d),
        new Coordinate(-85d, 0d),
        new Coordinate(-150.37d, 0d)
      );
    PathState thisState = PathState.getPathState(thisPath,
        VectorFactory.getDefault().createVector2D(
            -150.37d, -12.06d));
    
    final Vector difference = thisState.minus(otherState);
    
    assertEquals("dist", -121d, difference.getElement(0), 1d);
    assertEquals("vel", -14.65d, difference.getElement(1), 1d);
  }
  
  @Test
  public void testDistanceBetween17() {
    InferredPath startPath = makeTmpPath( 
        true, 
        new Coordinate(0, -262.21d),
        new Coordinate(0, 0),
        new Coordinate(0, 792.85d)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(
           -808.4862233549034, -26.2880263830998));
    
    InferredPath newPath = makeTmpPath( 
        true, 
        new Coordinate(0, 792.85d),
        new Coordinate(0, 0),
        new Coordinate(0, -262.21d)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    PathState stateOnNewPath = PathState.getPathState(newPath, result);
    final Vector difference = stateOnNewPath.minus(
       startState);
    
    assertTrue(difference.isZero(1e-7));
  }
  
  @Test
  public void testDistanceBetween18() {
    InferredPath otherPath = makeTmpPath( 
        true, 
        new Coordinate(-90d, 0d),
        new Coordinate(0d, 0d),
        new Coordinate(0d, -(517d-90d)/2d),
        new Coordinate(0d, 0d),
        new Coordinate(-90d, 0d)
      );
    
    PathState otherState = PathState.getPathState(otherPath,
        VectorFactory.getDefault().createVector2D(
            -564d, -8d));
    
    InferredPath thisPath = makeTmpPath( 
        false, 
        new Coordinate(0d, 0d),
        new Coordinate(-90d, 0d),
        new Coordinate(-90d, 607d-90d)
      );
    PathState thisState = PathState.getPathState(thisPath,
        VectorFactory.getDefault().createVector2D(
            47, 8d));
    
    final Vector difference = thisState.minus(otherState);
    
    assertEquals("dist", 0d, difference.getElement(0), 0d);
    assertEquals("vel", 0d, difference.getElement(1), 0d);
  }


  private InferredPath makeTmpPath(boolean isBackward, Coordinate ... coords) {
    return TrackingTestUtils.makeTmpPath(this.otpGraph, isBackward, coords);
  }
}
