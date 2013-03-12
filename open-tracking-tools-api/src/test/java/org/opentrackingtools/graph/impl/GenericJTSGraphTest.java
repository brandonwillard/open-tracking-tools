package org.opentrackingtools.graph.impl;

import org.testng.annotations.Test;
import org.testng.AssertJUnit;

import com.google.common.collect.Lists;

import com.vividsolutions.jcs.conflate.roads.RoadEdge;
import com.vividsolutions.jcs.conflate.roads.RoadEdgeMerger;
import com.vividsolutions.jcs.conflate.roads.RoadNetwork;

import com.vividsolutions.jts.algorithm.RobustLineIntersector;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.noding.IntersectionAdder;
import com.vividsolutions.jts.noding.MCIndexNoder;
import com.vividsolutions.jts.noding.NodedSegmentString;
import com.vividsolutions.jts.noding.SegmentString;
import com.vividsolutions.jts.noding.SegmentStringDissolver;
import com.vividsolutions.jts.noding.SegmentStringDissolver.SegmentStringMerger;

import org.geotools.geometry.jts.JTSFactoryFinder;

import com.vividsolutions.jump.feature.AttributeType;
import com.vividsolutions.jump.feature.BasicFeature;
import com.vividsolutions.jump.feature.Feature;
import com.vividsolutions.jump.feature.FeatureDataset;
import com.vividsolutions.jump.feature.FeatureSchema;

import gov.sandia.cognition.util.Pair;

import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.edges.PathEdge;

import java.util.Collections;
import java.util.Iterator;
import java.util.List;

public class GenericJTSGraphTest {

  @Test
  public void testGraphConstruction() {
//    MCIndexNoder ssd = new MCIndexNoder();
//    
//    ssd.setSegmentIntersector(new IntersectionAdder(new RobustLineIntersector()));
//    
//    ssd.computeNodes(Lists.newArrayList(
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(0,0), new Coordinate(2,0)}, "line4"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(0,0), new Coordinate(1,0)}, "line1"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(1,0), new Coordinate(1,-1)}, "line1"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(0,0), new Coordinate(1,0)}, "line2"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(1,0), new Coordinate(1,1)}, "line2"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(1,1), new Coordinate(1,0)}, "line3"),
//        new NodedSegmentString(
//            new Coordinate[] {new Coordinate(1,0), new Coordinate(0,0)}, "line3")
//        ));
//    
//    for (Object obj : ssd.getNodedSubstrings()) {
//      SegmentString ss = (SegmentString) obj;
//      System.out.println(java.util.Arrays.deepToString(ss.getCoordinates()));
//    }


    LineString l1 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
            new Coordinate(0,0),
            new Coordinate(1,0),
            new Coordinate(1,1),
        });
    
    LineString l2 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
            new Coordinate(0,0),
            new Coordinate(1,0),
            new Coordinate(1,-1),
        });
    
    LineString l3 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
            new Coordinate(0,0),
            new Coordinate(0.5,0),
            new Coordinate(0.5,-1),
        });
    
    LineString l4 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
            new Coordinate(1,-1),
            new Coordinate(1,0),
            new Coordinate(0,0),
      });
    
    LineString l5 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] {
            new Coordinate(1,1),
            new Coordinate(1,2),
            new Coordinate(1,3),
      });
    
    MCIndexNoder ssd2 = new MCIndexNoder();
    ssd2.setSegmentIntersector(new IntersectionAdder(new RobustLineIntersector()));
    ssd2.computeNodes(Lists.newArrayList(
        new NodedSegmentString(l1.getCoordinates(), "line1"), 
        new NodedSegmentString(l2.getCoordinates(), "line2"), 
        new NodedSegmentString(l3.getCoordinates(), "line3"), 
        new NodedSegmentString(l4.getCoordinates(), "line4"), 
        new NodedSegmentString(l5.getCoordinates(), "line4") 
        ));
    
    for (Object obj : ssd2.getNodedSubstrings()) {
      SegmentString ss2 = (SegmentString) obj;
      System.out.println("noded lines:" + java.util.Arrays.deepToString(ss2.getCoordinates()));
    }

    SegmentStringMerger merger = new SegmentStringMerger() {
      @Override
      public void merge(SegmentString mergeTarget, SegmentString ssToMerge,
          boolean isSameOrientation) {
        if (isSameOrientation)
          System.out.println("found opposite for " + mergeTarget);
      }
    };
    
    SegmentStringDissolver dissolver = new SegmentStringDissolver(merger);
    dissolver.dissolve(ssd2.getNodedSubstrings());
    
    for (Object obj : dissolver.getDissolved()) {
      SegmentString ss2 = (SegmentString) obj;
      System.out.println("dissolved lines:" + java.util.Arrays.deepToString(ss2.getCoordinates()));
    }

    final FeatureSchema schema = new FeatureSchema();
    schema.addAttribute("line", AttributeType.STRING);
    schema.addAttribute("geometry", AttributeType.GEOMETRY);
    
    Feature l1f = new BasicFeature(schema);
    l1f.setAttribute("line", "1");
    l1f.setGeometry(l1);
    Feature l2f = new BasicFeature(schema);
    l2f.setAttribute("line", "2");
    l2f.setGeometry(l2);
    
//    Feature l3f = new BasicFeature(schema);
//    l3f.setAttribute("line", "3");
//    l3f.setGeometry(l3);
//    graph.addEdge(l3f);
//    Feature l4f = new BasicFeature(schema);
//    l4f.setAttribute("line", "4");
//    l4f.setGeometry(l4);
//    graph.addEdge(l4f);
    
    FeatureDataset fc = new FeatureDataset(Lists.newArrayList(l1f, l2f), schema);
    RoadNetwork graph = new RoadNetwork(fc);
    graph.addEdge(l1, Collections.singletonList(l1f));
    graph.addEdge(l2, Collections.singletonList(l2f));
    
    graph.index();
    RoadEdgeMerger rem = new RoadEdgeMerger();
    rem.merge(graph);
    
    for (Object obj : graph.getEdges()) {
      RoadEdge edge = (RoadEdge)obj;
      System.out.println(edge.getGeometry());
    }
    
    System.out.println("done");
  }
  
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
