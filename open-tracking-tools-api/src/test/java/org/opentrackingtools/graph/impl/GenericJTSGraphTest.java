package org.opentrackingtools.graph.impl;

import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import com.beust.jcommander.internal.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;

import gov.sandia.cognition.util.Pair;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.edges.PathEdge;

import java.util.List;

public class GenericJTSGraphTest {

  @Test
  public void testGetPathFromGraph() {
    
    List<LineString> lines = Lists.newArrayList();
    LineString l1 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0, 0), new Coordinate(0.5, 0.5), 
            new Coordinate(1, 0.5), new Coordinate(1, 1)});
    LineString l2 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(1, 1), new Coordinate(1.5, 1.5), 
            new Coordinate(2, 1.5), new Coordinate(2, 2)});
    lines.add(l1);
    lines.add(l2);
    GenericJTSGraph graph = new GenericJTSGraph(lines, false);
    
    InferredEdge infEdge1 = graph.getInferredEdge("2");
    LengthIndexedSubline startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0, 0), new Coordinate(0.5, 0.5)),
            infEdge1);
    LengthIndexedSubline endEdge = startEdge; 
    
    Pair<List<PathEdge>, Double> pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge1, 0d, false);
    
    /*
     * Test end edge same as start
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0, 0), new Coordinate(0.5, 0.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    
    
    startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)),
            infEdge1);
    endEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0, 0), new Coordinate(0.5, 0.5)),
            infEdge1); 
    
    pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge1, 0d, false);
    /*
     * Test end edge before start
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    
    InferredEdge infEdge2 = graph.getInferredEdge("4");
    
    startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0, 0), new Coordinate(0.5, 0.5)),
            infEdge1);
    endEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(1, 1), new Coordinate(1.5, 1.5)),
            infEdge2); 
    
    pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge1, 0d, false);
    
    /*
     * Test when end edge isn't on start edge.
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0, 0), new Coordinate(0.5, 0.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)}), 
        pathEdgePair.getFirst().get(1).getGeometry());
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(1, 0.5), new Coordinate(1, 1)}), 
        pathEdgePair.getFirst().get(2).getGeometry());
    
    startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0, 0), new Coordinate(0.5, 0.5)),
            infEdge1);
    endEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0.5, 0.5), new Coordinate(1, 1.5)),
            infEdge1); 
    
    pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge1, 0d, false);
    
    /*
     * Test sub section of edge (first two segments)
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0, 0), new Coordinate(0.5, 0.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)}), 
        pathEdgePair.getFirst().get(1).getGeometry());
    
    startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)),
            infEdge1);
    endEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(1, 0.5), new Coordinate(1, 1)),
            infEdge1); 
    
    pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge1, 0d, false);
    
    /*
     * Test sub section of edge (last two segments)
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(0.5, 0.5), new Coordinate(1, 0.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(1, 0.5), new Coordinate(1, 1)}), 
        pathEdgePair.getFirst().get(1).getGeometry());
    
    startEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(0, 0), new Coordinate(0, 0.5)),
            infEdge1);
    endEdge = new LengthIndexedSubline(
        new LineSegment(new Coordinate(1.5, 1.5), new Coordinate(2, 1.5)),
            infEdge2); 
    
    pathEdgePair = 
        graph.getPathEdges(startEdge, endEdge, infEdge2, 0d, false);
    
    /*
     * Test start not on line
     */
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(1, 1), new Coordinate(1.5, 1.5)}), 
        pathEdgePair.getFirst().get(0).getGeometry());
    AssertJUnit.assertEquals(JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {new Coordinate(1.5, 1.5), new Coordinate(2, 1.5)}), 
        pathEdgePair.getFirst().get(1).getGeometry());
    
  }

}
