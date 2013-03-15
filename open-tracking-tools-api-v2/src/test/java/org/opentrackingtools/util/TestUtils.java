package org.opentrackingtools.util;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.mockito.Mockito.stub;

import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.geotools.referencing.CRS;
import org.opengis.feature.Feature;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;

public class TestUtils {

  public static LineString makeGeometry(Coordinate v0,
    Coordinate v1) {
    final GeometryFactory gf = new GeometryFactory();
    final Coordinate[] coordinates =
        new Coordinate[] { v0, v1 };
    return gf.createLineString(coordinates);
  }

  public static Path makeTmpPath(InferenceGraph graph,
    boolean isBackward, Coordinate... coords) {
    Preconditions.checkArgument(coords.length > 1);

    
    final List<InferenceGraphEdge> edges = Lists.newArrayList();
    Coordinate lastCoord = null;
    for (int i = 0; i < coords.length; i++) {
      final Coordinate coord = coords[i];
//      final StreetVertex vt =
//          new IntersectionVertex(graph.getBaseGraph(),
//              "tmpVertex" + i, coord, "none");
      if (lastCoord != null) {
        final double distance =
            coord.distance(lastCoord);
        final LineString geom =
            JTSFactoryFinder.getGeometryFactory().createLineString(
                new Coordinate[] {lastCoord, coord});
        
        stub(graph.edgeHasReverse(geom)).toReturn(false);
        
        final InferenceGraphEdge ie =
            new InferenceGraphEdge(geom, geom, 1000 + i, graph);
        
        
        edges.add(ie);
      }
      lastCoord = coord;
    }

    if (isBackward) {
      Collections.reverse(edges);
    }

    final List<PathEdge> pathEdges = Lists.newArrayList();
    double distToStart = 0;
    for (final InferenceGraphEdge edge : edges) {
      for (InferenceGraphSegment segment :edge.getSegments()) {
        final PathEdge pe =
            new PathEdge(segment, (isBackward ? -1d : 1d)
                * distToStart, isBackward);
        distToStart += edge.getLength();
        pathEdges.add(pe);
      }
    }

    return new Path(pathEdges,
        isBackward);
  }

  public static List<LineString> createGridGraph(Coordinate startCoord) throws IOException, NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException {
    List<LineString> edges = Lists.newArrayList();
    
    /*
     * Create a grid
     */
    CoordinateReferenceSystem crs = 
        CRS.getAuthorityFactory(false).createGeographicCRS("EPSG:4326");
    
    Envelope tmpEnv = new Envelope(startCoord);
    
    final double appMeter = GeoUtils.getMetersInAngleDegrees(1d);
    tmpEnv.expandBy(appMeter * 10e3);
    
    ReferencedEnvelope gridBounds = new ReferencedEnvelope(
       tmpEnv.getMinX() ,tmpEnv.getMaxX() , 
       tmpEnv.getMinY() , tmpEnv.getMaxY(), crs);
    
    List<OrthoLineDef> lineDefs = Arrays.asList(
        // vertical (longitude) lines
        new OrthoLineDef(LineOrientation.VERTICAL, 1, appMeter * 100d),
        // horizontal (latitude) lines
        new OrthoLineDef(LineOrientation.HORIZONTAL, 1, appMeter * 100d)
        );

    SimpleFeatureSource grid = Lines.createOrthoLines(gridBounds, lineDefs);
    FeatureIterator iter = grid.getFeatures().features();
   
    List<Object> geoms = Lists.newArrayList();
    while (iter.hasNext()) {
      Feature feature = iter.next();
      LineString geom = (LineString)feature.getDefaultGeometryProperty().getValue();
      /*
       * XXX
       * Using Object so that the correct equals (equalsExact) will be called?
       */
      geoms.add(geom);
    }
    
    Geometry superGeom  = JTSFactoryFinder.getGeometryFactory().buildGeometry(geoms).union();
    
    for (int i = 0; i < ((MultiLineString)superGeom).getNumGeometries(); i++) {
      if (superGeom.getGeometryN(i) instanceof LineString) {
        LineString geom = (LineString) superGeom.getGeometryN(i);
        if (geom.getLength() > 1e-5) {
          edges.add(geom);
          edges.add((LineString)geom.reverse());
        }
      }
    }
    
    return edges;
  }

}
