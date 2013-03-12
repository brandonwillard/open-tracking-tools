package org.opentrackingtools.graph.paths.util;


import org.testng.AssertJUnit;
import org.testng.annotations.Test;
import org.testng.annotations.DataProvider;
import org.testng.internal.junit.ArrayAsserts;

import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.graph.build.line.DirectedLineStringGraphGenerator;
import org.geotools.graph.structure.basic.BasicDirectedEdge;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.statistics.distributions.impl.AdjMultivariateGaussian;
import org.opentrackingtools.util.GeoUtils;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.algorithm.LineIntersector;
import com.vividsolutions.jts.algorithm.RobustLineIntersector;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geomgraph.DirectedEdge;
import com.vividsolutions.jts.geomgraph.Edge;
import com.vividsolutions.jts.geomgraph.EdgeIntersection;
import com.vividsolutions.jts.geomgraph.GeometryGraph;
import com.vividsolutions.jts.geomgraph.Label;
import com.vividsolutions.jts.geomgraph.index.EdgeSetIntersector;
import com.vividsolutions.jts.geomgraph.index.SegmentIntersector;
import com.vividsolutions.jts.geomgraph.index.SimpleEdgeSetIntersector;
import com.vividsolutions.jts.geomgraph.index.SimpleMCSweepLineIntersector;
import com.vividsolutions.jts.noding.BasicSegmentString;
import com.vividsolutions.jts.noding.SegmentString;
import com.vividsolutions.jts.noding.SegmentStringDissolver;
import com.vividsolutions.jts.noding.SegmentStringUtil;
import com.vividsolutions.jts.operation.GeometryGraphOperation;
import com.vividsolutions.jts.operation.linemerge.LineMergeEdge;
import com.vividsolutions.jts.operation.linemerge.LineMergeGraph;
import com.vividsolutions.jts.operation.linemerge.LineMerger;
import com.vividsolutions.jts.operation.overlay.EdgeSetNoder;

public class PathUtilsTest {
  
  @DataProvider
  private static final Object[][] stateData() {
   
    return new Object[][] {
    {VectorFactory.getDefault().copyArray(
            new double[] {-9.999992857195395E7, 0.0, -13.537576549229717, 0.0}),
            1e8,
      VectorFactory.getDefault().copyArray(new double[] {-9.999992857195395E7 - 1e8, 0.0}),
      VectorFactory.getDefault().copyArray(new double[] {-9.999992857195395E7, 0d, 0d, 0.0})
      },
    {VectorFactory.getDefault().copyArray(
            new double[] {-9.999992857195395E1, 0.0, -13.537576549229717, 0.0}), 
            1e2,
      VectorFactory.getDefault().copyArray(new double[] {-9.999992857195395E1 - 1e2, 0.0}),
      VectorFactory.getDefault().copyArray(new double[] {-9.999992857195395E1, 0d, 0d, 0.0})
      }
    };
  }

  @Test(dataProvider="stateData")
  public void testProjection(Vector from, double length, Vector roadTo, Vector groundTo) {
    
    InferenceGraph graph = mock(InferenceGraph.class);
    final InferredPath path =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(-length, 0d),
            new Coordinate(0d, 0d),
            new Coordinate(length, 0d));
    
    MultivariateGaussian belief = new AdjMultivariateGaussian(
        from, MatrixFactory.getDefault().copyArray(
                new double[][] {
                    {91.64766085510277, 0.0, -10.790534809853966, 0.0},
                    {0.0, 0.0, 0.0, 0.0},
                    {-10.790534809853973, 0.0, 110.08645314343424, 0.0},
                    {0.0, 0.0, 0.0, 0.0}
                })
            );
    
    final PathEdge pathEdge = Iterables.getLast(path.getPathEdges());
    MultivariateGaussian projBelief = PathUtils.getRoadBeliefFromGround(belief, 
        path.getGeometry(), path.isBackward(), 
        pathEdge.isBackward() ? pathEdge.getGeometry().reverse() : pathEdge.getGeometry(), 
        pathEdge.getDistToStartOfEdge(), true);
    
   
    ArrayAsserts.assertArrayEquals("convert to road", 
        roadTo.toArray(), 
        projBelief.getMean().toArray(), 1e-1);
    
    PathUtils.convertToGroundBelief(projBelief, pathEdge, false, true);
    
    ArrayAsserts.assertArrayEquals("convert back to ground", 
        groundTo.toArray(), 
        projBelief.getMean().toArray(), 1e-1);
  }
  
//  @Test
  public void geomMerge() {
    GeometryFactory gf = JTSFactoryFinder.getGeometryFactory();
    LineString l1 = gf.createLineString(new Coordinate[] {
        new Coordinate(0, 0), 
        new Coordinate(0, 0.5),
        new Coordinate(0, 1), 
        new Coordinate(1, 1)
        });
    com.vividsolutions.jts.geom.LineString l2 = gf.createLineString(new Coordinate[] {
        new Coordinate(0, 0), 
        new Coordinate(0, 0.5),
        new Coordinate(0, 1), 
        new Coordinate(-1, 1)
        });
    
    BasicSegmentString ss1 = new BasicSegmentString(
        l1.getCoordinates(), "line1");
    BasicSegmentString ss2 = new BasicSegmentString(
        l2.getCoordinates(), "line2");
    
    SegmentStringDissolver dissolver = new SegmentStringDissolver();
    List<LineSegment> dsegs = Lists.newArrayList();
    dsegs.addAll(GeoUtils.getSubLineSegments(l1));
    dsegs.addAll(GeoUtils.getSubLineSegments(l2));
    List<SegmentString> toDissolve = Lists.newArrayList();
    for (LineSegment seg : dsegs) {
      toDissolve.add(new BasicSegmentString(new Coordinate[] {seg.p0, seg.p1}, null));
    }
    
//    toDissolve.addAll(SegmentStringUtil.extractSegmentStrings(l1));
//    toDissolve.addAll(SegmentStringUtil.extractSegmentStrings(l2));
    dissolver.dissolve(toDissolve);
    
    Collection<SegmentString> dissolved = dissolver.getDissolved();
    for (SegmentString ss : dissolved) {
      System.out.println(ss.getCoordinates());
    }
    
    DirectedLineStringGraphGenerator graphGenerator = new DirectedLineStringGraphGenerator();
    
    graphGenerator.add(l1);
    graphGenerator.add(l2);
    
    LineMergeGraph lmg = new LineMergeGraph();
    lmg.addEdge(l1);
    lmg.addEdge(l2);
    
    LineMerger lm = new LineMerger();
    lm.add(l1);
    lm.add(l2);
    Collection<LineString> merged = lm.getMergedLineStrings();
   
    for (Object edge : lmg.getEdges()) {
      LineMergeEdge dEdge = (LineMergeEdge) edge;
      System.out.println(dEdge.getLine());
    }
    
    Edge de1 = new Edge(l1.getCoordinates(), new Label(0));
    de1.setName("edge1");
    Edge de2 = new Edge(l2.getCoordinates(), new Label(1));
    de2.setName("edge2");
    LineIntersector li = new RobustLineIntersector();
    SegmentIntersector si = new SegmentIntersector(li, true, true);
    EdgeSetIntersector esi = new SimpleEdgeSetIntersector(); 
    esi.computeIntersections(Lists.newArrayList(de1, de2), si, false);
    List<Edge> splits = Lists.newArrayList();
    de1.getEdgeIntersectionList().addSplitEdges(splits);
    de2.getEdgeIntersectionList().addSplitEdges(splits);
    List<LineString> splitsRes = Lists.newArrayList();
    for (Edge edge : splits) {
      splitsRes.add(gf.createLineString(edge.getCoordinates()));
    }
    
    EdgeSetIntersector esi2 = new SimpleMCSweepLineIntersector();
    LineIntersector li2 = new RobustLineIntersector();
    SegmentIntersector si2 = new SegmentIntersector(li2, false, false);
    esi2.computeIntersections(Lists.newArrayList(de1, de2), si2, false);
    List<Edge> splits2 = Lists.newArrayList();
    de1.getEdgeIntersectionList().addSplitEdges(splits2);
    de2.getEdgeIntersectionList().addSplitEdges(splits2);
    List<LineString> splitsRes2 = Lists.newArrayList();
    for (Edge edge : splits2) {
      splitsRes2.add(gf.createLineString(edge.getCoordinates()));
    }
    
    AssertJUnit.assertTrue(false);
    
  }


}
